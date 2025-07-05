#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class GamepadParser : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new GamepadParser node.
   *
   * Declares the "deadzone" parameter and sets up the subscriber for the /joy topic
   * and the publisher for the /desired_twist topic.
   */
  GamepadParser();

private:
  /**
   * @brief Callback function for processing incoming joystick messages.
   *
   * Applies a deadzone to joystick axes and publishes a geometry_msgs::msg::Twist message
   * to the /desired_twist topic.
   *
   * @param msg Shared pointer to the received sensor_msgs::msg::Joy message.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  std::map<std::string, uint8_t> axis_bindings_;
  std::map<std::string, uint8_t> button_bindings_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};