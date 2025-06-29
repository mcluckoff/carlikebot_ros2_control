// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carlikebot_ros2_control/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace carlikebot_ros2_control
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.CarlikeBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
  cfg_.front_left_steering_name = info_.hardware_parameters["front_left_steering_name"];
  cfg_.front_right_steering_name = info_.hardware_parameters["front_right_steering_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);  
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);  
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  wheel_rl_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rr_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      wheel_rl_.name, hardware_interface::HW_IF_POSITION, &wheel_rl_.vel));
      
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      wheel_rr_.name, hardware_interface::HW_IF_POSITION, &wheel_rr_.vel));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos));
      
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos));      


  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          wheel_rl_.name, hardware_interface::HW_IF_POSITION,
          &wheel_rl_.cmd));
  command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          wheel_rr_.name, hardware_interface::HW_IF_POSITION,
          &wheel_rr_.cmd));
  command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          wheel_fl_.name, hardware_interface::HW_IF_POSITION,
          &wheel_fl_.cmd));
  command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          wheel_fr_.name, hardware_interface::HW_IF_POSITION,
          &wheel_fr_.cmd));                        
  
  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  comms_.set_pid_values(20,15,1,25);
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  comms_.read_encoder_values(wheel_rl_.enc, wheel_rr_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_rl_.pos;
  wheel_rl_.pos = wheel_rl_.calcEncAngle();
  wheel_rl_.vel = (wheel_rl_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_rr_.pos;
  wheel_rr_.pos = wheel_rr_.calcEncAngle();
  wheel_rr_.vel = (wheel_rr_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type carlikebot_ros2_control ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int motor_rl_counts_per_loop = wheel_rl_.cmd / wheel_rl_.rads_per_count / cfg_.loop_rate;
  int motor_rr_counts_per_loop = wheel_rr_.cmd / wheel_rr_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_rl_counts_per_loop, motor_rr_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace carlikebot_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  carlikebot_ros2_control::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
