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
  // if (info_.joints.size() != 6)
  // {
  //   RCLCPP_ERROR(
  //     get_logger(),
  //     "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
  //     "because the number of joints %ld is not 6.",
  //     info_.joints.size());
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  if (info_.joints.size() < 4 || info_.joints.size() > 6)
  {
    RCLCPP_ERROR(
      get_logger(),
      "CarlikeBotSystemHardware::on_init() - Expected 4..6 joints, got %ld.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  front_steer_names_.clear();
  rear_drive_names_.clear();
  front_spin_names_.clear();

  for (const auto &j : info_.joints)
{
  const auto &name = j.name;
  if (is_front_steer(name))      front_steer_names_.push_back(name);
  else if (is_rear_drive(name))  rear_drive_names_.push_back(name);
  else if (is_front_spin(name))  front_spin_names_.push_back(name);
  else
  {
    RCLCPP_FATAL(get_logger(), "Unknown joint by naming: '%s'", name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

// --- Sanity checks ---
if (front_steer_names_.size() != 2)
{
  RCLCPP_FATAL(get_logger(), "Need exactly 2 front steering joints, found %zu.",
               front_steer_names_.size());
  return hardware_interface::CallbackReturn::ERROR;
}
if (rear_drive_names_.size() != 2)
{
  RCLCPP_FATAL(get_logger(), "Need exactly 2 rear drive wheel joints, found %zu.",
               rear_drive_names_.size());
  return hardware_interface::CallbackReturn::ERROR;
}
// front_spin_names_ may be 0 or 2 (two passive front wheel joints)
if (!(front_spin_names_.empty() || front_spin_names_.size() == 2))
{
  RCLCPP_FATAL(get_logger(), "Front spin (passive) joints must be 0 or 2, found %zu.",
               front_spin_names_.size());
  return hardware_interface::CallbackReturn::ERROR;
}

// --- Interface-level validation per group ---
auto validate_front_steer = [&](const hardware_interface::ComponentInfo &j) {
  if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    return false;
  if (j.state_interfaces.size() != 1 ||
      j.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    return false;
  return true;
};
auto validate_rear_drive = [&](const hardware_interface::ComponentInfo &j) {
  if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    return false;
  if (j.state_interfaces.size() != 2) return false;
  bool has_pos=false, has_vel=false;
  for (auto &si : j.state_interfaces) {
    has_pos |= (si.name == hardware_interface::HW_IF_POSITION);
    has_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
  }
  return has_pos && has_vel;
};
auto validate_front_spin = [&](const hardware_interface::ComponentInfo &j) {
  if (!j.command_interfaces.empty()) return false; // passive: no command
  bool has_pos=false;
  for (auto &si : j.state_interfaces) {
    has_pos |= (si.name == hardware_interface::HW_IF_POSITION);
  }
  return has_pos; // position state required; velocity optional
};

for (const auto &j : info_.joints)
{
  if (is_front_steer(j.name) && !validate_front_steer(j)) {
    RCLCPP_FATAL(get_logger(), "Steering joint '%s' has wrong interfaces.", j.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (is_rear_drive(j.name) && !validate_rear_drive(j)) {
    RCLCPP_FATAL(get_logger(), "Rear drive joint '%s' has wrong interfaces.", j.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (is_front_spin(j.name) && !validate_front_spin(j)) {
    RCLCPP_FATAL(get_logger(), "Front spin (passive) joint '%s' has wrong interfaces.", j.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

  cfg_.front_left_wheel_joint_name = info_.hardware_parameters["front_left_wheel_joint_name"];
  cfg_.front_right_wheel_joint_name = info_.hardware_parameters["front_right_wheel_joint_name"];
  cfg_.rear_left_wheel_joint_name = info_.hardware_parameters["rear_left_wheel_joint_name"];  
  cfg_.rear_right_wheel_joint_name = info_.hardware_parameters["rear_right_wheel_joint_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev_traction = std::stoi(info_.hardware_parameters["enc_counts_per_rev_traction"]);
  cfg_.enc_counts_per_rev_steering = std::stoi(info_.hardware_parameters["enc_counts_per_rev_steering"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "PID values not supplied, using defaults.");
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  // hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

  // hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

  // Steering (front) — use names from parameters to preserve your steering_left/right semantics
  // cfg_.front_left_wheel_joint_name is actually a steering joint name in your config
  hw_interfaces_["steering_left"]  =
    Joint(cfg_.front_left_wheel_joint_name,  cfg_.enc_counts_per_rev_steering);
  hw_interfaces_["steering_right"] =
    Joint(cfg_.front_right_wheel_joint_name, cfg_.enc_counts_per_rev_steering);

  // Rear drive
  hw_interfaces_["traction_left"]  =
    Joint(cfg_.rear_left_wheel_joint_name,   cfg_.enc_counts_per_rev_traction);
  hw_interfaces_["traction_right"] =
    Joint(cfg_.rear_right_wheel_joint_name,  cfg_.enc_counts_per_rev_traction);

  // Passive front spin joints (optional, use real joint names as keys)
  for (const auto &name : front_spin_names_) {
    hw_interfaces_[name] = Joint(name, 1 /*dummy enc per rev*/);
  }

  if (!front_spin_names_.empty()) {
    RCLCPP_INFO(get_logger(), "Passive front spin joints enabled: %s, %s",
                front_spin_names_[0].c_str(), front_spin_names_[1].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first.find("traction") != std::string::npos)
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first.find("steering") != std::string::npos)
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_POSITION,
          &joint.second.command.position));
    }
    else if (joint.first.find("traction") != std::string::npos)
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.command.velocity));
    }
  }

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
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first.find("traction") != std::string::npos)
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first.find("steering") != std::string::npos)
    {
      joint.second.command.position = 0.0;
    }
  }

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(hw_interfaces_["traction_right"].enc, hw_interfaces_["traction_left"].enc, 
                            hw_interfaces_["steering_left"].enc, hw_interfaces_["steering_right"].enc);

  double delta_seconds = period.seconds();

  double pos_prev = hw_interfaces_["traction_left"].state.position;
  hw_interfaces_["traction_left"].state.position = hw_interfaces_["traction_left"].calc_enc_angle();
  hw_interfaces_["traction_left"].state.velocity = (hw_interfaces_["traction_left"].state.position - pos_prev) / delta_seconds;

  pos_prev = hw_interfaces_["traction_right"].state.position;
  hw_interfaces_["traction_right"].state.position = hw_interfaces_["traction_right"].calc_enc_angle();
  hw_interfaces_["traction_right"].state.velocity = (hw_interfaces_["traction_right"].state.position - pos_prev) / delta_seconds;

  hw_interfaces_["steering_left"].state.position = hw_interfaces_["steering_left"].calc_enc_angle();
  hw_interfaces_["steering_right"].state.position = hw_interfaces_["steering_right"].calc_enc_angle();

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  // hw_interfaces_["steering_left"].state.position = hw_interfaces_["steering_left"].command.position;
  // hw_interfaces_["steering_right"].state.position = hw_interfaces_["steering_right"].command.position;

  // hw_interfaces_["traction_left"].state.velocity = hw_interfaces_["traction_left"].command.velocity;
  // hw_interfaces_["traction_left"].state.position +=
  //   hw_interfaces_["traction_left"].state.velocity * period.seconds();

  // hw_interfaces_["traction_right"].state.velocity = hw_interfaces_["traction_right"].command.velocity;
  // hw_interfaces_["traction_right"].state.position +=
  //   hw_interfaces_["traction_right"].state.velocity * period.seconds();

  // --- Synthesize passive front spin states so TF shows up in RViz ---
  const double delta_seconds = period.seconds();
  const double omega_avg = 0.5 * (
    hw_interfaces_["traction_left"].state.velocity +
    hw_interfaces_["traction_right"].state.velocity);

  auto update_passive = [&](const std::string& name) {
    auto it = hw_interfaces_.find(name);
    if (it == hw_interfaces_.end()) return;
    it->second.state.velocity = omega_avg;
    it->second.state.position += omega_avg * delta_seconds;
    // Optional wrapping:
    // it->second.state.position = std::remainder(it->second.state.position, 2*M_PI);
  };

for (const auto &name : front_spin_names_) {
  update_passive(name);
}

  std::stringstream ss;
  ss << "Reading states:";

  ss << std::fixed << std::setprecision(2) << std::endl
     << "\t" << "position: " << hw_interfaces_["steering_left"].state.position
     << " (enc: " << hw_interfaces_["steering_left"].enc << ")"
     << " for joint '" << hw_interfaces_["steering_left"].joint_name.c_str() << "'" << std::endl

     << "\t" << "position: " << hw_interfaces_["steering_right"].state.position
     << " (enc: " << hw_interfaces_["steering_right"].enc << ")"
     << " for joint '" << hw_interfaces_["steering_right"].joint_name.c_str() << "'" << std::endl

     << "\t" << "position: " << hw_interfaces_["traction_left"].state.position
     << " (enc: " << hw_interfaces_["traction_left"].enc << ")"
     << " for joint '" << hw_interfaces_["traction_left"].joint_name.c_str() << "'" << std::endl

     << "\t" << "velocity: " << hw_interfaces_["traction_left"].state.velocity
     << " for joint '" << hw_interfaces_["traction_left"].joint_name.c_str() << "'" << std::endl

     << "\t" << "position: " << hw_interfaces_["traction_right"].state.position
     << " (enc: " << hw_interfaces_["traction_right"].enc << ")"
     << " for joint '" << hw_interfaces_["traction_right"].joint_name.c_str() << "'" << std::endl

     << "\t" << "velocity: " << hw_interfaces_["traction_right"].state.velocity
     << " for joint '" << hw_interfaces_["traction_right"].joint_name.c_str() << "'";

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type carlikebot_ros2_control ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // int traction_motor_l_counts_per_loop = hw_interfaces_["traction_left"].command.velocity / hw_interfaces_["traction_left"].rad_per_enc_count / cfg_.loop_rate;
  int traction_motor_r_counts_per_loop = hw_interfaces_["traction_right"].command.velocity / hw_interfaces_["traction_right"].rad_per_enc_count / cfg_.loop_rate;
  comms_.set_motor_values(traction_motor_r_counts_per_loop);

  // int steering_l_target_enc_count = hw_interfaces_["steering_left"].command.position / hw_interfaces_["steering_left"].rad_per_enc_count;
  int steering_r_target_enc_count = hw_interfaces_["steering_right"].command.position / hw_interfaces_["steering_right"].rad_per_enc_count;
  comms_.set_steering_values(steering_r_target_enc_count);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  ss << std::fixed << std::setprecision(2) << std::endl

     << "\t" << "position: " << hw_interfaces_["steering_left"].command.position
     << " (target enc: " << steering_r_target_enc_count << ")"
     << " for joint '" << hw_interfaces_["steering_left"].joint_name.c_str() << "'" << std::endl

     << "\t" << "position: " << hw_interfaces_["steering_right"].command.position
     << " (target enc: " << steering_r_target_enc_count << ")"
     << " for joint '" << hw_interfaces_["steering_right"].joint_name.c_str() << "'" << std::endl

     << "\t" << "velocity: " << hw_interfaces_["traction_left"].command.velocity
      // << " (counts/loop: " << traction_motor_r_counts_per_loop << ")"
     << " for joint '" << hw_interfaces_["traction_left"].joint_name.c_str() << "'" << std::endl

     << "\t" << "velocity: " << hw_interfaces_["traction_right"].command.velocity
      // << " (counts/loop: " << traction_motor_r_counts_per_loop << ")"
     << " for joint '" << hw_interfaces_["traction_right"].joint_name.c_str() << "'";

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace carlikebot_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  carlikebot_ros2_control::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
