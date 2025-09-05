/**
 * @file EKF_sensor_fusion_controller.hpp
 * @brief Extended Kalman Filter for 6-DOF State Estimation with IMU, Pressure Sensor, and USBL
 *
 * This filter estimates the full 6-DOF state of a vehicle by fusing data from:
 * - A 9-DOF IMU (e.g., BNO055) providing linear acceleration, angular velocity, and orientation (quaternion).
 * - A pressure sensor providing depth (z-position).
 * - A USBL system providing noisy position fixes, with distance-dependent measurement uncertainty.
 *
 * @section state_vector State Definition
 *
 * The EKF state vector is:
 * @f[
 * \mathbf{x}_k =
 * \begin{bmatrix}
 * \mathbf{p}_k \\
 * \mathbf{v}_k \\
 * \mathbf{q}_k \\
 * \bm{\omega}_k \\
 * \mathbf{b}_{\bm{\omega}_k} \\
 * \mathbf{b}_{\mathbf{a}_k}
 * \end{bmatrix}
 * \in \mathbb{R}^{19}
 * @f]
 *
 * where:
 * - @f$ \mathbf{p}_k \in \mathbb{R}^3 @f$: Position in the world frame.
 * - @f$ \mathbf{v}_k \in \mathbb{R}^3 @f$: Velocity in the world frame.
 * - @f$ \mathbf{q}_k \in \mathbb{R}^4 @f$: Unit quaternion (orientation from body to world frame).
 * - @f$ \bm{\omega}_k \in \mathbb{R}^3 @f$: Angular velocity in the body frame.
 * - @f$ \mathbf{b}_{\bm{\omega}_k} \in \mathbb{R}^3 @f$: Gyro bias.
 * - @f$ \mathbf{b}_{\mathbf{a}_k} \in \mathbb{R}^3 @f$: Accelerometer bias.
 *
 * @section process_model Process Model
 *
 * The discrete-time motion model over timestep @f$ \Delta t @f$ is:
 *
 * @f[
 * \begin{aligned}
 * \mathbf{p}_{k+1} &= \mathbf{p}_k + \mathbf{v}_k \Delta t + @frac{1}{2}(\mathbf{R}(\mathbf{q}_k)(\mathbf{a}_k-\mathbf{b}_{\mathbf{a}_k}) - \mathbf{g}) \Delta t^2 \\
 * \mathbf{v}_{k+1} &= \mathbf{v}_k + (\mathbf{R}(\mathbf{q}_k)(\mathbf{a}_k-\mathbf{b}_{\mathbf{a}_k}) - \mathbf{g}) \Delta t - \mathbf{D}_{\text{linear}}\mathbf{v}_k\Delta t \\
 * \mathbf{q}_{k+1} &= \mathbf{q}_k \otimes \exp_q\left( @frac{1}{2} (\bm{\omega}_k-\mathbf{b}_{\bm{\omega}_k}) \Delta t \right) \\
 * \bm{\omega}_{k+1} &= \bm{\omega}_k - \mathbf{D}_{\text{angular}}\bm{\omega}_k\Delta t + \mathbf{w}_{\bm{\omega}} \\
 * \mathbf{b}_{\bm{\omega}_{k+1}} &= \mathbf{b}_{\bm{\omega}_k} + \mathbf{w}_b \\
 * \mathbf{b}_{\mathbf{a}_{k+1}} &= \mathbf{b}_{\mathbf{a}_k} + \mathbf{w}_{ba}
 * \end{aligned}
 * @f]
 *
 * where:
 * - @f$ \mathbf{a}_k @f$: IMU-measured linear acceleration (body frame).
 * - @f$ \mathbf{R}(\mathbf{q}) @f$: Rotation matrix corresponding to @f$ \mathbf{q} @f$.
 * - @f$ \exp_q(\cdot) @f$: Quaternion exponential map.
 * - @f$ \mathbf{g} @f$: World-frame gravity vector.
 * - @f$ \otimes @f$: Quaternion multiplication.
 * - @f$ \mathbf{w}_{\bm{\omega}} @f$: Angular velocity random walk noise.
 * - @f$ \mathbf{w}_b @f$: Gyro bias random walk noise.
 * - @f$ \mathbf{w}_{ba} @f$: Accelerometer bias random walk noise.
 * - @f$ \mathbf{D}_{\text{linear}} = \text{diag}(d_x, d_y, d_z) @f$: Linear drag coefficients.
 * - @f$ \mathbf{D}_{\text{angular}} = \text{diag}(d_{\text{roll}}, d_{\text{pitch}}, d_{\text{yaw}}) @f$: Angular drag coefficients.
 *
 * @section hydrodynamic_model Hydrodynamic Effects Model
 * 
 * The filter incorporates a simplified hydrodynamic model to account for drag forces in underwater environments:
 * 
 * @f[
 * \begin{aligned}
 * \mathbf{F}_{\text{drag}} &= -\mathbf{D}_{\text{linear}}\mathbf{v}_k \\
 * \bm{\tau}_{\text{drag}} &= -\mathbf{D}_{\text{angular}}\bm{\omega}_k
 * \end{aligned}
 * @f]
 * 
 * where:
 * - @f$ \mathbf{F}_{\text{drag}} @f$: Drag force in world frame.
 * - @f$ \bm{\tau}_{\text{drag}} @f$: Drag torque in body frame.
 * - @f$ \mathbf{D}_{\text{linear}} @f$ and @f$ \mathbf{D}_{\text{angular}} @f$: Diagonal matrices of linear and angular drag coefficients.
 * 
 * This model captures the first-order effects of water resistance, which is particularly important for accurate velocity estimation 
 * and improved dead-reckoning performance during periods without external position measurements.
 *
 * @section quaternion_expmap Quaternion Exponential Map
 *
 * Used to update orientation from angular velocity:
 * @f[
 * \delta \mathbf{q}_k = \exp_q\left( @frac{1}{2} \bm{\omega}_k \Delta t \right)
 * = \left[ \cos\left(@frac{\theta}{2}\right),\ \sin\left(@frac{\theta}{2}\right) @frac{\bm{\omega}_k}{\theta} \right]
 * \quad \text{where} \quad \theta = \|\bm{\omega}_k\| \Delta t
 * @f]
 *
 * For small @f$ \theta @f$, use first-order approximation:
 * @f$ \delta \mathbf{q}_k \approx [1,\ @frac{1}{2} \bm{\omega}_k \Delta t] @f$
 *
 * @section measurement_models Measurement Models
 *
 * - **Pressure Sensor (depth only):** @f$ z_k = p_k^{(z)} + v_k @f$
 * - **USBL Position Measurement:** @f$ z_k^{\text{USBL}} = p_k + v_k^{\text{noise}} @f$
 *
 * @section measurement_gating Measurement Validation Gating
 * 
 * A statistical validation gate is applied to reject outlier measurements:
 * https://rtificer.github.io/ROV2026/EKF__sensor__fusion__controller_8hpp.html
 * @f[
 * d^2 = \mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k \leq \gamma
 * @f]
 * 
 * where:
 * - @f$ \mathbf{y}_k = \mathbf{z}_k - h(\mathbf{x}_{k|k-1}) @f$ is the innovation (measurement residual)
 * - @f$ \mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k @f$ is the innovation covariance
 * - @f$ d^2 @f$ is the squared Mahalanobis distance
 * - @f$ \gamma @f$ is the chi-square threshold for the given confidence level and measurement dimension
 * 
 * Measurements that fail this test are considered outliers and are not used in the update step.
 * This is especially important for USBL measurements, which can occasionally suffer from multipath effects,
 * interference, or other anomalies that lead to highly erroneous position fixes.
 *
 * @section usb_cov USBL Position Measurement Model and Range-Dependent Noise Covariance
 *
 * The USBL (Ultra-Short Baseline) system provides an indirect position fix.
 *
 * @subsection range_dependent_cov Range-Dependent Measurement Noise Covariance
 *
 * Due to the USBL's reliance on angle-of-arrival sensing, measurement error increases with range.
 * A small angular error @f$ \delta\theta @f$ corresponds to larger position uncertainty at greater distances.
 *
 * @subsection geometric_interp Geometric Interpretation
 *
 * The vehicle's true position lies on a sphere of radius @f$ r_k = \|p_k - p_{USBL}\| @f$.
 * Angular error introduces tangent-plane uncertainty; range error affects the radial direction.
 *
 * @subsection covariance_model Covariance Modeling
 *
 * The USBL measurement noise covariance matrix is modeled as:
 *
 * @f[
 * R_k^{\text{USBL}} = \sigma_\theta^2 r_k^2 (I_3 - \mathbf{n}\mathbf{n}^T) + \sigma_r^2 \mathbf{n}\mathbf{n}^T
 * @f]
 *
 * where:
 * - @f$ \sigma_\theta^2 @f$: Angle-of-arrival variance.
 * - @f$ \sigma_r^2 @f$: Range measurement variance.
 * - @f$ \mathbf{n} = @frac{\mathbf{p}_k - \mathbf{p}_{USBL}}{\|\mathbf{p}_k - \mathbf{p}_{USBL}\|} @f$: Line-of-sight unit vector.
 * - @f$ I_3 - \mathbf{n}\mathbf{n}^T @f$: Projects onto the tangent plane of the measurement sphere.
 *
 * @subsection covariance_intuition Intuition
 *
 * - @f$ \sigma_\theta^2 r_k^2 (I - nn^T) @f$ captures angular uncertainty (perpendicular).
 * - @f$ \sigma_r^2 nn^T @f$ captures radial uncertainty (along line-of-sight).
 *
 * This results in an ellipsoidal measurement covariance aligned with the sensor geometry.
 *
 * @section accelerometer_bias Accelerometer Bias Estimation
 * 
 * The filter estimates accelerometer bias @f$ \mathbf{b}_{\mathbf{a}} @f$ to compensate for systematic errors:
 * 
 * @f[
 * \begin{aligned}
 * \mathbf{a}_{\text{true}} &= \mathbf{a}_{\text{measured}} - \mathbf{b}_{\mathbf{a}} \\
 * \mathbf{b}_{\mathbf{a}_{k+1}} &= \mathbf{b}_{\mathbf{a}_k} + \mathbf{w}_{ba}
 * \end{aligned}
 * @f]
 * 
 * where @f$ \mathbf{w}_{ba} \sim \mathcal{N}(0, \mathbf{Q}_{ba}) @f$ models the bias as a random walk process.
 * 
 * Bias estimation is critical because:
 * - Small biases integrate into significant velocity errors over time
 * - They distinguish true accelerations from sensor offsets
 * - They compensate for temperature-dependent drift and manufacturing imperfections
 * 
 * Since accelerometer biases couple with gravity in the world frame, they're best observable 
 * during maneuvers with varying orientation relative to gravity.
 *
 * @section ekf_update EKF Measurement Update
 *
 * During each EKF update:
 * - The range @f$ r_k @f$ is recomputed from the predicted state.
 * - The USBL measurement covariance @f$ R_k^{USBL} @f$ is recalculated.
 * - Measurement Jacobians @f$ H_k @f$ are constructed for:
 *     - USBL: @f$ H_k^{USBL} = [I_3,\ 0_{3x16}] @f$
 *     - Pressure: @f$ H_k^{depth} = [0\ 0\ 1\ \mathbf{0}_{1 \times 16}] @f$
 *
 * @section jacobians Jacobian Computation
 *
 * The EKF linearizes the process and measurement models:
 * - State transition Jacobian @f$ F_k = @frac{\partial f}{\partial x} @f$
 * - Measurement Jacobian @f$ H_k = @frac{\partial h}{\partial x} @f$
 *
 * Orientation derivatives use quaternion product Jacobians:
 *
 * Left quaternion multiplication matrix:
 * @f[
 * L(\mathbf{q}) =
 * \begin{bmatrix}
 * q_w & -q_x & -q_y & -q_z \\
 * q_x &  q_w & -q_z &  q_y \\
 * q_y &  q_z &  q_w & -q_x \\
 * q_z & -q_y &  q_x &  q_w
 * \end{bmatrix}
 * @f]
 *
 * And the derivative of rotated vectors:
 * @f[
 * @frac{\partial (\mathbf{R}(\mathbf{q}) \mathbf{a})}{\partial \mathbf{q}} \in \mathbb{R}^{3 \times 4}
 * @f]
 * gives sensitivity of the rotated IMU accelerations to orientation changes.
 *
 * These Jacobians are essential for accurately propagating uncertainty in nonlinear systems.
 */