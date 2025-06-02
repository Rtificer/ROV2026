#include "rov_control/gamepad_parser_node.hpp"


GamepadParser::GamepadParser() : Node("gamepad_parser")
{
  this->declare_parameter<float>("deadzone", 0.1f);

  // Parse axes
  std::vector<std::string> axis_names;
  this->get_parameter("axes", axis_names);
  for (const auto& axis_name : axis_names) {
    std::string binding;
    int index;
    this->get_parameter("axes." + axis_name + ".binding", binding);
    this->get_parameter("axes." + axis_name + ".index", index);
    if (!binding.empty()) {
      axis_bindings_[binding] = index;
    }
  }

  // Parse buttons
  std::vector<std::string> button_names;
  this->get_parameter("buttons", button_names);
  for (const auto& button_name : button_names) {
    std::string binding;
    int index;
    this->get_parameter("buttons." + button_name + ".binding", binding);
    this->get_parameter("buttons." + button_name + ".index", index);
    if (!binding.empty()) {
      button_bindings_[binding] = index;
    }
  }

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&GamepadParser::joy_callback, this, std::placeholders::_1));
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/desired_velocity", 10);
}

void GamepadParser::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  float deadzone = this->get_parameter("deadzone").as_double();

  /**
   * @brief Apply a deadzone to a joystick axis value.
   *
   * If the absolute value of the input is less than the deadzone threshold,
   * returns 0.0f; otherwise, returns the original value.
   *
   * @param value The joystick axis value.
   * @param deadzone The deadzone threshold.
   * @return float The processed axis value after applying the deadzone.
   */
  auto apply_deadzone = [](float value, float deadzone) {
    return (std::abs(value) < deadzone) ? 0.0f : value;
  };

  geometry_msgs::msg::Twist twist;
  // Use binding names to get the correct axis index
  twist.linear.x  = apply_deadzone(msg->axes[axis_bindings_["surge"]], deadzone);
  twist.linear.y  = apply_deadzone(msg->axes[axis_bindings_["sway"]], deadzone);
  twist.linear.z  = apply_deadzone(msg->axes[axis_bindings_["heave"]], deadzone);
  twist.angular.z = apply_deadzone(msg->axes[axis_bindings_["yaw"]], deadzone);
  twist.angular.x = apply_deadzone(msg->axes[axis_bindings_["pitch"]], deadzone);
  twist.angular.y = apply_deadzone(msg->axes[axis_bindings_["roll"]], deadzone);

  // Example: check if a button with a binding is pressed
  // if (button_bindings_.count("some_action")) {
  //   bool pressed = msg->buttons[button_bindings_["some_action"]];
  //   // ... handle button press ...
  // }

  twist_pub_->publish(twist);
}

/**
 * @brief Main entry pouint8_t for the gamepad parser node.
 *
 * Initializes ROS, spins the GamepadParser node, and shuts down ROS on exit.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return uint8_t Exit code.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadParser>());
  rclcpp::shutdown();
  return 0;
}