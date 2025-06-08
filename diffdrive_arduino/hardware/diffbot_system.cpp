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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.door_servo_name = info_.hardware_parameters["door_servo_name"];
  cfg_.ramp_servo_name = info_.hardware_parameters["ramp_servo_name"];
  cfg_.collector_servo_name = info_.hardware_parameters["collector_servo_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  door_servo_.setup(cfg_.door_servo_name);
  ramp_servo_.setup(cfg_.ramp_servo_name);
  collector_servo_.setup(cfg_.collector_servo_name);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    door_servo_.name, hardware_interface::HW_IF_POSITION, &door_servo_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    door_servo_.name, hardware_interface::HW_IF_VELOCITY, &door_servo_.vel));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    ramp_servo_.name, hardware_interface::HW_IF_POSITION, &ramp_servo_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    ramp_servo_.name, hardware_interface::HW_IF_VELOCITY, &ramp_servo_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    collector_servo_.name, hardware_interface::HW_IF_POSITION, &collector_servo_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    collector_servo_.name, hardware_interface::HW_IF_VELOCITY, &collector_servo_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    door_servo_.name, hardware_interface::HW_IF_VELOCITY, &door_servo_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    ramp_servo_.name, hardware_interface::HW_IF_VELOCITY, &ramp_servo_.cmd));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    collector_servo_.name, hardware_interface::HW_IF_VELOCITY, &collector_servo_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");


  // ─── create our private node and IMU publisher ───────────────────

  nh_ = std::make_shared<rclcpp::Node>("diffdrive_arduino_hw_node");
  imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data_raw", rclcpp::QoS(10));

  RCLCPP_INFO(
  rclcpp::get_logger("DiffDriveArduinoHardware"),
  "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //wheel_l_.pos = 0.0;
  //wheel_r_.pos = 0.0;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  //bool wheel_l_neg = false;
  //bool wheel_r_neg = false;
  //if (wheel_l_.cmd < 0.0) wheel_l_neg =true;
  //if (wheel_r_.cmd < 0.0) wheel_r_neg = true;

  //wheel_l_.cmd = std::abs(wheel_l_.cmd);
  //wheel_r_.cmd = std::abs(wheel_r_.cmd);

  //wheel_l_.vel = (wheel_l_.cmd-24.5) / 16.52;
  //wheel_r_.vel = (wheel_r_.cmd-24.5) / 16.52;

  comms_.get_values(wheel_l_.vel, wheel_r_.vel);

  double ax, ay, az, gx, gy, gz;

  comms_.get_imu(ax, ay, az, gx, gy, gz);
  auto now = nh_->now();

  imu_msg_.header.stamp = now;
  imu_msg_.header.frame_id = "imu_link";
  imu_msg_.linear_acceleration.x = ax;
  imu_msg_.linear_acceleration.y = ay;
  imu_msg_.linear_acceleration.z = az;
  imu_msg_.angular_velocity.x = gx;
  imu_msg_.angular_velocity.y = gy;
  imu_msg_.angular_velocity.z = gz;
  imu_msg_.orientation_covariance[0] = -1;  // indicate “unknown”

  imu_pub_->publish(imu_msg_);

  double delta_seconds = period.seconds();

  //if (wheel_l_neg) wheel_l_.vel = -wheel_l_.vel;
  //if (wheel_r_neg) wheel_r_.vel = -wheel_r_.vel;

  //if(wheel_l_.cmd < 24.5)
  //{
  //  wheel_l_.vel = 0.0;
  //}
  //if(wheel_r_.cmd < 24.5)
  //{
  //  wheel_r_.vel = 0.0;
  //}

  //allo woohooo
  // lots of code
  wheel_r_.pos = wheel_r_.vel*delta_seconds + wheel_r_.pos;
  wheel_l_.pos = wheel_l_.vel*delta_seconds + wheel_l_.pos;

  //RCLCPP_INFO(
   //rclcpp::get_logger("DiffDriveArduinoHardware"), "Read motor values: %f %f", wheel_l_.vel, wheel_r_.vel);
  
  int i = 0;
  int e = 0;
  int f = 0;

  //RCLCPP_INFO(
   //rclcpp::get_logger("DiffDriveArduinoHardware"), "Read positions values: %f %f", wheel_l_.vel,  wheel_r_.vel);
  
  door_servo_.vel = 1;
  door_servo_.pos = 1;

  ramp_servo_.vel = 1;
  ramp_servo_.pos = 1;

  collector_servo_.vel = 1;
  collector_servo_.pos = 1;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  //RCLCPP_INFO(
    //rclcpp::get_logger("DiffDriveArduinoHardware"), "Sent Inital motor values: %f %f", wheel_l_.cmd, wheel_r_.cmd);
  //bool wheel_l_neg = false;
  //bool wheel_r_neg = false;
  //if (wheel_l_.cmd < 0.0) wheel_l_neg =true;
  //if (wheel_r_.cmd < 0.0) wheel_r_neg = true;

  //wheel_l_.cmd = std::abs(wheel_l_.cmd);
  //wheel_r_.cmd = std::abs(wheel_r_.cmd);
  wheel_l_.cmd = wheel_l_.cmd*60.0*60.0/(2*M_PI);
  wheel_r_.cmd = wheel_r_.cmd*60.0*60.0/(2*M_PI)*1.03;

    //RCLCPP_INFO(
    //rclcpp::get_logger("DiffDriveArduinoHardware"), "Sent motor values: %f %f", wheel_l_.cmd, wheel_r_.cmd);
  //if (wheel_l_neg) wheel_l_.cmd = -wheel_l_.cmd;
  //if (wheel_r_neg) wheel_r_.cmd = -wheel_r_.cmd;
  comms_.set_motor_values(wheel_l_.cmd, wheel_r_.cmd);

  comms_.set_servo_door_values(door_servo_.cmd);

  comms_.set_servo_ramp_values(ramp_servo_.cmd);

  comms_.set_servo_collector_values(collector_servo_.cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
