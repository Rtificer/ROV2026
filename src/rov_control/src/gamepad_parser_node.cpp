#include "rov_control/gamepad_parser_node.hpp"


GamepadParser::GamepadParser() : Node("gamepad_parser")
{
  this->declare_parameter<float>("deadzone", 0.1f); // Declare parameter with default value

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
  twist.linear.x = apply_deadzone(msg->axes[1], deadzone);   // Left stick vertical
  twist.linear.y = apply_deadzone(msg->axes[0], deadzone);   // Left stick horizontal
  twist.angular.z = apply_deadzone(msg->axes[3], deadzone);  // Right stick horizontal
  twist_pub_->publish(twist);
}

/**
 * @brief Main entry point for the gamepad parser node.
 *
 * Initializes ROS, spins the GamepadParser node, and shuts down ROS on exit.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadParser>());
  rclcpp::shutdown();
  return 0;
}