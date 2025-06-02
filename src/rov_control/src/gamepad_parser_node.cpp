#include "rov_control/gamepad_parser_node.hpp"


GamepadParser::GamepadParser() : Node("gamepad_parser")
{
  //List of possible axis keys for the gamepad
  std::vector<std::string> axis_keys = {
    "left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y",
    "left_trigger", "right_trigger", "dpad_x", "dpad_y"
  };
  for (const auto& axis_key : axis_keys) {
    std::string binding;
    uint8_t index;
    this->declare_parameter<std::string>("axes." + axis_key + ".binding", "");
    this->declare_parameter<uint8_t>("axes." + axis_key + ".index", -1);
    this->get_parameter("axes." + axis_key + ".binding", binding);
    this->get_parameter("axes." + axis_key + ".index", index);
    if (!binding.empty() && index >= 0) {
      axis_bindings_[binding] = index;
    }
  }

  //List of possible button keys for the gamepad
  std::vector<std::string> button_keys = {
    "a", "b", "x", "y", "left_bumper", "right_bumper",
    "left_stick_button", "right_stick_button", "menu_button",
    "view_button", "upload_button", "xbox_button"
  };
  for (const auto& button_key : button_keys) {
    std::string binding;
    uint8_t index;
    this->declare_parameter<std::string>("buttons." + button_key + ".binding", "");
    this->declare_parameter<uint8_t>("buttons." + button_key + ".index", -1);
    this->get_parameter("buttons." + button_key + ".binding", binding);
    this->get_parameter("buttons." + button_key + ".index", index);
    if (!binding.empty() && index >= 0) {
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
  // Removed: float deadzone = this->get_parameter("deadzone").as_double();

  geometry_msgs::msg::Twist twist;
  // Use binding names to get the correct axis index
  twist.linear.x  = msg->axes[axis_bindings_["surge"]];
  twist.linear.y  = msg->axes[axis_bindings_["sway"]];
  twist.linear.z  = msg->axes[axis_bindings_["heave"]];
  twist.angular.z = msg->axes[axis_bindings_["yaw"]];
  twist.angular.x = msg->axes[axis_bindings_["pitch"]];
  twist.angular.y = msg->axes[axis_bindings_["roll"]];

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