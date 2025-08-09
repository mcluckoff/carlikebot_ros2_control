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

#ifndef CARLIKEBOT_ROS2_CONTROL__CARLIKEBOT_SYSTEM_HPP_
#define CARLIKEBOT_ROS2_CONTROL__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_comms.hpp"

namespace carlikebot_ros2_control
{
struct Config
{
  std::string front_left_wheel_joint_name = "";
  std::string front_right_wheel_joint_name = "";
  std::string rear_left_wheel_joint_name = "";
  std::string rear_right_wheel_joint_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev_traction = 0;
  int enc_counts_per_rev_steering = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  explicit Joint(const std::string & name, int enc_counts_per_rev) : joint_name(name)
  {
    state = JointValue();
    command = JointValue();

    rad_per_enc_count = (2 * M_PI) / enc_counts_per_rev;
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;

  int enc = 0;
  double rad_per_enc_count = 0;

  double calc_enc_angle()
  {
    return enc * rad_per_enc_count;
  }
};
class CarlikeBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  // --- Name-based classification helpers (simple find-based) ---
  inline static bool is_front(const std::string& n)  { return n.find("front") != std::string::npos; }
  inline static bool is_rear (const std::string& n)  { return n.find("rear")  != std::string::npos; }
  inline static bool is_steer(const std::string& n)  { return n.find("steer") != std::string::npos; }
  inline static bool is_wheel(const std::string& n)  { return n.find("wheel") != std::string::npos; }
  inline static bool is_left (const std::string& n)  { return n.find("left")  != std::string::npos; }
  inline static bool is_right(const std::string& n)  { return n.find("right") != std::string::npos; }

  // Convenience predicates
  inline static bool is_front_steer(const std::string& n) { return is_front(n) && is_steer(n); }
  inline static bool is_rear_drive(const std::string& n)  { return is_rear(n)  && is_wheel(n); }
  inline static bool is_front_spin(const std::string& n)  { return is_front(n) && is_wheel(n) && !is_steer(n); }

  // --- Grouped joint name lists filled at on_init() ---
  std::vector<std::string> front_steer_names_;   // e.g., front_left_steering_joint, front_right_steering_joint
  std::vector<std::string> rear_drive_names_;    // e.g., rear_left_wheel_joint, rear_right_wheel_joint
  std::vector<std::string> front_spin_names_;    // e.g., front_left_wheel_joint, front_right_wheel_joint

  ArduinoComms comms_;
  Config cfg_;

  // Parameters for the CarlikeBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // std::vector<std::tuple<std::string, double, double>>
  //   hw_interfaces_;  // name of joint, state, command
  std::map<std::string, Joint> hw_interfaces_;
};

}  // namespace carlikebot_ros2_control

#endif  // CARLIKEBOT_ROS2_CONTROL__CARLIKEBOT_SYSTEM_HPP_
