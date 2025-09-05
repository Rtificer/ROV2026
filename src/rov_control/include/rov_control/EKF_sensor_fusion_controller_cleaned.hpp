/**
 * @file EKF_sensor_fusion_controller.hpp
 * @brief Extended Kalman Filter for 6-DOF State Estimation with IMU, Pressure Sensor, and USBL
 *
 * This filter estimates the full 6-DOF state of an underwater vehicle by fusing data from:
 * - A 9-DOF IMU (e.g., BNO055) providing linear acceleration, angular velocity, and orientation (quaternion)
 * - A pressure sensor providing depth (z-position) measurements
 * - A USBL system providing noisy position fixes with distance-dependent measurement uncertainty
 *
 * The filter incorporates underwater-specific models including hydrodynamic drag effects and bias estimation
 * for improved accuracy in challenging marine environments.
 *
 * @section state_vector State Vector Definition
 *
 * The EKF state vector consists of 19 elements representing the vehicle's complete kinematic state:
 *
 * \f[
 * \mathbf{x}_k =
 * \begin{bmatrix}
 * \mathbf{p}_k \\
 * \mathbf{v}_k \\
 * \mathbf{q}_k \\
 * \mathbf{\omega}_k \\
 * \mathbf{b}_{\mathbf{\omega}_k} \\
 * \mathbf{b}_{\mathbf{a}_k}
 * \end{bmatrix}
 * \in \mathbb{R}^{19}
 * \f]
 *
 * where:
 * - \f$ \mathbf{p}_k \in \mathbb{R}^3 \f$: Position in the world frame
 * - \f$ \mathbf{v}_k \in \mathbb{R}^3 \f$: Velocity in the world frame
 * - \f$ \mathbf{q}_k \in \mathbb{R}^4 \f$: Unit quaternion representing orientation from body to world frame
 * - \f$ \mathbf{\omega}_k \in \mathbb{R}^3 \f$: Angular velocity in the body frame
 * - \f$ \mathbf{b}_{\mathbf{\omega}_k} \in \mathbb{R}^3 \f$: Gyroscope bias vector
 * - \f$ \mathbf{b}_{\mathbf{a}_k} \in \mathbb{R}^3 \f$: Accelerometer bias vector
 *
 * This comprehensive state representation allows the filter to track both translational and rotational motion
 * while simultaneously estimating sensor biases that can drift over time due to environmental factors.
 *
 * @section process_model Process Model and Quaternion Integration
 *
 * The discrete-time motion model propagates the state over timestep \f$ \Delta t \f$ using physics-based equations:
 *
 * \f[
 * \begin{aligned}
 * \mathbf{p}_{k+1} &= \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}(\mathbf{R}(\mathbf{q}_k)(\mathbf{a}_k-\mathbf{b}_{\mathbf{a}_k}) - \mathbf{g}) \Delta t^2 \\
 * \mathbf{v}_{k+1} &= \mathbf{v}_k + (\mathbf{R}(\mathbf{q}_k)(\mathbf{a}_k-\mathbf{b}_{\mathbf{a}_k}) - \mathbf{g}) \Delta t - \mathbf{D}_{\text{linear}}\mathbf{v}_k\Delta t \\
 * \mathbf{q}_{k+1} &= \mathbf{q}_k \otimes \exp_q\left( \frac{1}{2} (\mathbf{\omega}_k-\mathbf{b}_{\mathbf{\omega}_k}) \Delta t \right) \\
 * \mathbf{\omega}_{k+1} &= \mathbf{\omega}_k - \mathbf{D}_{\text{angular}}\mathbf{\omega}_k\Delta t + \mathbf{w}_{\mathbf{\omega}} \\
 * \mathbf{b}_{\mathbf{\omega}_{k+1}} &= \mathbf{b}_{\mathbf{\omega}_k} + \mathbf{w}_b \\
 * \mathbf{b}_{\mathbf{a}_{k+1}} &= \mathbf{b}_{\mathbf{a}_k} + \mathbf{w}_{ba}
 * \end{aligned}
 * \f]
 *
 * The quaternion update uses the exponential map for numerically stable integration:
 * \f[
 * \delta \mathbf{q}_k = \exp_q\left( \frac{1}{2} \mathbf{\omega}_k \Delta t \right)
 * = \left[ \cos\left(\frac{\theta}{2}\right),\ \sin\left(\frac{\theta}{2}\right) \frac{\mathbf{\omega}_k}{\theta} \right]
 * \]
 * where \f$ \theta = \|\mathbf{\omega}_k\| \Delta t \f$. For small angles, the first-order approximation
 * \f$ \delta \mathbf{q}_k \approx [1,\ \frac{1}{2} \mathbf{\omega}_k \Delta t] \f$ is used to avoid singularities.
 *
 * Model parameters:
 * - \f$ \mathbf{a}_k \f$: IMU-measured linear acceleration (body frame)
 * - \f$ \mathbf{R}(\mathbf{q}) \f$: Rotation matrix corresponding to quaternion \f$ \mathbf{q} \f$
 * - \f$ \mathbf{g} \f$: World-frame gravity vector \f$ [0, 0, 9.81]^T \f$
 * - \f$ \otimes \f$: Quaternion multiplication operator
 * - \f$ \mathbf{w}_{\mathbf{\omega}}, \mathbf{w}_b, \mathbf{w}_{ba} \f$: Process noise terms for angular velocity and bias random walks
 * - \f$ \mathbf{D}_{\text{linear}} = \text{diag}(d_x, d_y, d_z) \f$: Linear drag coefficient matrix
 * - \f$ \mathbf{D}_{\text{angular}} = \text{diag}(d_{\text{roll}}, d_{\text{pitch}}, d_{\text{yaw}}) \f$: Angular drag coefficient matrix
 *
 * @section hydrodynamic_model Hydrodynamic Drag Effects
 * 
 * The filter incorporates simplified hydrodynamic drag models essential for underwater vehicle dynamics:
 * 
 * \f[
 * \begin{aligned}
 * \mathbf{F}_{\text{drag}} &= -\mathbf{D}_{\text{linear}}\mathbf{v}_k \\
 * \mathbf{\tau}_{\text{drag}} &= -\mathbf{D}_{\text{angular}}\mathbf{\omega}_k
 * \end{aligned}
 * \f]
 * 
 * where \f$ \mathbf{F}_{\text{drag}} \f$ represents drag force in the world frame and \f$ \mathbf{\tau}_{\text{drag}} \f$ 
 * represents drag torque in the body frame. The diagonal drag matrices \f$ \mathbf{D}_{\text{linear}} \f$ and 
 * \f$ \mathbf{D}_{\text{angular}} \f$ capture first-order hydrodynamic resistance effects.
 * 
 * This modeling is critical for underwater applications because:
 * - Water resistance significantly affects vehicle dynamics compared to aerial vehicles
 * - Drag forces provide natural damping that improves velocity estimation accuracy
 * - Proper drag modeling enhances dead-reckoning performance during USBL outages
 * - The model helps distinguish between thruster-induced and drag-induced accelerations
 *
 * @section measurement_models Measurement Models and Sensor Integration
 *
 * The filter processes measurements from multiple sensors with distinct characteristics:
 *
 * **Pressure Sensor (Depth Measurement):**
 * \f[
 * z_k^{\text{depth}} = p_k^{(z)} + v_k^{\text{depth}}
 * \f]
 * where \f$ p_k^{(z)} \f$ is the z-component of position and \f$ v_k^{\text{depth}} \sim \mathcal{N}(0, \sigma_{\text{depth}}^2) \f$.
 * Pressure sensors provide highly accurate depth information with minimal drift, serving as a crucial
 * absolute reference for vertical position estimation.
 *
 * **USBL Position Measurement:**
 * \f[
 * z_k^{\text{USBL}} = \mathbf{p}_k + \mathbf{v}_k^{\text{USBL}}
 * \f]
 * where \f$ \mathbf{v}_k^{\text{USBL}} \sim \mathcal{N}(0, \mathbf{R}_k^{\text{USBL}}) \f$ with range-dependent covariance.
 * USBL measurements provide full 3D position but with variable accuracy depending on distance and
 * environmental conditions.
 *
 * **IMU Integration:**
 * The IMU provides continuous measurements of linear acceleration and angular velocity, which are
 * integrated through the process model. Quaternion measurements from the IMU can optionally be
 * incorporated as direct orientation observations when available.
 *
 * @section measurement_gating Statistical Measurement Validation
 * 
 * A chi-square statistical validation gate rejects outlier measurements to improve robustness:
 * 
 * \f[
 * d^2 = \mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k \leq \gamma
 * \f]
 * 
 * where:
 * - \f$ \mathbf{y}_k = \mathbf{z}_k - h(\mathbf{x}_{k|k-1}) \f$ is the innovation (measurement residual)
 * - \f$ \mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k \f$ is the innovation covariance matrix
 * - \f$ d^2 \f$ is the squared Mahalanobis distance
 * - \f$ \gamma \f$ is the chi-square threshold corresponding to the desired confidence level and measurement dimension
 * 
 * This validation is particularly important for USBL measurements, which can suffer from:
 * - Multipath propagation effects in complex underwater environments
 * - Acoustic interference from other systems or marine life
 * - Signal blockage by obstacles or vehicle structures
 * - Range-dependent degradation in measurement quality
 * 
 * Measurements failing the validation test are rejected, allowing the filter to rely on dead-reckoning
 * until valid measurements become available again.
 *
 * @section usbl_covariance USBL Range-Dependent Noise Modeling
 *
 * USBL systems exhibit measurement uncertainty that increases with range due to their angle-of-arrival
 * sensing principle. The measurement noise covariance matrix models this geometric relationship:
 *
 * \f[
 * \mathbf{R}_k^{\text{USBL}} = \sigma_\theta^2 r_k^2 (\mathbf{I}_3 - \mathbf{n}\mathbf{n}^T) + \sigma_r^2 \mathbf{n}\mathbf{n}^T
 * \f]
 *
 * where:
 * - \f$ \sigma_\theta^2 \f$: Angular measurement variance (constant)
 * - \f$ \sigma_r^2 \f$: Range measurement variance (constant)
 * - \f$ r_k = \|\mathbf{p}_k - \mathbf{p}_{\text{USBL}}\| \f$: Range from USBL transducer to vehicle
 * - \f$ \mathbf{n} = \frac{\mathbf{p}_k - \mathbf{p}_{\text{USBL}}}{\|\mathbf{p}_k - \mathbf{p}_{\text{USBL}}\|} \f$: Unit vector along line-of-sight
 * - \f$ \mathbf{I}_3 - \mathbf{n}\mathbf{n}^T \f$: Projection onto the tangent plane perpendicular to line-of-sight
 *
 * This formulation captures the geometric nature of USBL uncertainty:
 * - The term \f$ \sigma_\theta^2 r_k^2 (\mathbf{I}_3 - \mathbf{n}\mathbf{n}^T) \f$ models angular uncertainty, which increases
 *   quadratically with range and acts perpendicular to the line-of-sight
 * - The term \f$ \sigma_r^2 \mathbf{n}\mathbf{n}^T \f$ models range uncertainty, which is independent of distance
 *   and acts along the line-of-sight direction
 *
 * This results in an ellipsoidal uncertainty region that is elongated perpendicular to the line-of-sight
 * at longer ranges, accurately reflecting the physical limitations of acoustic positioning systems.
 *
 * @section bias_estimation Sensor Bias Estimation and Observability
 * 
 * The filter estimates both gyroscope and accelerometer biases to compensate for systematic sensor errors:
 * 
 * \f[
 * \begin{aligned}
 * \mathbf{\omega}_{\text{true}} &= \mathbf{\omega}_{\text{measured}} - \mathbf{b}_{\mathbf{\omega}} \\
 * \mathbf{a}_{\text{true}} &= \mathbf{a}_{\text{measured}} - \mathbf{b}_{\mathbf{a}} \\
 * \mathbf{b}_{\mathbf{\omega}_{k+1}} &= \mathbf{b}_{\mathbf{\omega}_k} + \mathbf{w}_b \\
 * \mathbf{b}_{\mathbf{a}_{k+1}} &= \mathbf{b}_{\mathbf{a}_k} + \mathbf{w}_{ba}
 * \end{aligned}
 * \f]
 * 
 * where \f$ \mathbf{w}_b, \mathbf{w}_{ba} \sim \mathcal{N}(0, \mathbf{Q}_b) \f$ model the biases as random walk processes.
 * 
 * Bias estimation is essential because:
 * - Small systematic errors integrate into large position and orientation drifts over time
 * - Temperature variations and aging affect sensor calibration
 * - Manufacturing tolerances create unit-to-unit variations in sensor characteristics
 * - Proper bias estimation significantly improves long-term navigation accuracy
 * 
 * **Observability Considerations:**
 * - Gyroscope biases are observable through orientation drift when external attitude references are available
 * - Accelerometer biases are most observable during vehicle maneuvers with varying orientation relative to gravity
 * - The coupling between accelerometer bias and gravity requires sufficient vehicle motion for proper estimation
 * - USBL position fixes provide the external reference needed to observe IMU bias effects
 *
 * @section ekf_implementation EKF Implementation and Jacobian Computation
 *
 * The Extended Kalman Filter linearizes the nonlinear process and measurement models using Jacobian matrices:
 *
 * **State Transition Jacobian:**
 * \f$ \mathbf{F}_k = \frac{\partial f(\mathbf{x}_k, \mathbf{u}_k)}{\partial \mathbf{x}_k} \f$ captures how small changes in the
 * current state propagate to the next time step.
 *
 * **Measurement Jacobians:**
 * - USBL: \f$ \mathbf{H}_k^{\text{USBL}} = [\mathbf{I}_3,\ \mathbf{0}_{3 \times 16}] \f$ (direct position observation)
 * - Pressure: \f$ \mathbf{H}_k^{\text{depth}} = [0\ 0\ 1\ \mathbf{0}_{1 \times 16}] \f$ (z-position only)
 *
 * **Quaternion Jacobians:**
 * For orientation updates, the left quaternion multiplication matrix is used:
 * \f[
 * \mathbf{L}(\mathbf{q}) =
 * \begin{bmatrix}
 * q_w & -q_x & -q_y & -q_z \\
 * q_x &  q_w & -q_z &  q_y \\
 * q_y &  q_z &  q_w & -q_x \\
 * q_z & -q_y &  q_x &  q_w
 * \end{bmatrix}
 * \f]
 *
 * The derivative of body-to-world rotated vectors with respect to quaternion parameters:
 * \f[
 * \frac{\partial (\mathbf{R}(\mathbf{q}) \mathbf{a})}{\partial \mathbf{q}} \in \mathbb{R}^{3 \times 4}
 * \f]
 * provides the sensitivity of rotated IMU measurements to orientation changes, enabling proper
 * uncertainty propagation through the nonlinear rotation operations.
 *
 * **Update Sequence:**
 * During each filter cycle:
 * 1. Predict state and covariance using the process model
 * 2. Compute range-dependent USBL measurement covariance \f$ \mathbf{R}_k^{\text{USBL}} \f$
 * 3. Apply measurement validation gating to incoming sensor data
 * 4. Update state estimate using validated measurements
 * 5. Maintain quaternion normalization to prevent numerical drift
 */
