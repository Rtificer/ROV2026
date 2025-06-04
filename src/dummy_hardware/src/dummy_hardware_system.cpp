#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>
#include <string>

using hardware_interface::return_type;
using hardware_interface::SystemInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

class DummyHardwareSystem : public SystemInterface
{
public:
  DummyHardwareSystem() = default;

  hardware_interface::CallbackReturn on_init(const HardwareInfo & info) override
  {
    info_ = info;
    states_.resize(info.joints.size(), 0.0);
    commands_.resize(info.joints.size(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &states_[i]);
    }
    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &commands_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    states_ = commands_;
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return return_type::OK;
  }

private:
  HardwareInfo info_;
  std::vector<double> states_;
  std::vector<double> commands_;
};

PLUGINLIB_EXPORT_CLASS(DummyHardwareSystem, hardware_interface::SystemInterface)