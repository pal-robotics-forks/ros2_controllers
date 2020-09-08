// Copyright 2020 PAL Robotics S.L.
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

#include <limits>
#include <memory>
#include <utility>

#include "control_toolbox/pid_ros.hpp"
#include "velocity_controllers/single_joint_position_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace
{
constexpr auto kSJPCLoggerName = "single joint position controller";
}

namespace velocity_controllers
{
using CallbackReturn = SingleJointPositionController::CallbackReturn;

SingleJointPositionController::SingleJointPositionController()
: SingleJointVelocityController(),
  pid_(nullptr),
  joint_state_handle_(nullptr),
  period_(std::numeric_limits<double>::quiet_NaN()),
  pid_duration_(rclcpp::Duration::max())
{
  logger_name_ = kSJPCLoggerName;
}

CallbackReturn SingleJointPositionController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // configure PID
  pid_ = std::make_shared<control_toolbox::PidROS>(
    lifecycle_node_->get_node_base_interface(),
    lifecycle_node_->get_node_logging_interface(),
    lifecycle_node_->get_node_parameters_interface(),
    lifecycle_node_->get_node_topics_interface(), "pid");

  if (!pid_->initPid()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "failed to initialize PID");
    return CallbackReturn::ERROR;
  }

  /// @todo should period be part of the controller interface?
  // get period parameter
  rclcpp::Parameter period_param;
  if (!lifecycle_node_->get_parameter("period", period_param)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "'period' parameter not set");
    return CallbackReturn::ERROR;
  }

  period_ = period_param.as_double();
  if (period_ <= 0.0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "'period' invalid value");
    return CallbackReturn::ERROR;
  }

  pid_duration_ = rclcpp::Duration::from_seconds(period_);

  // configure base controller
  if (SingleJointVelocityController::on_configure(previous_state) == CallbackReturn::ERROR) {
    return CallbackReturn::ERROR;
  }

  // get joint state handle
  if (auto rh_ptr = robot_hardware_.lock()) {
    // at this point we know the joint parameter is available
    const auto joint_name = lifecycle_node_->get_parameter("joint").as_string();
    hardware_interface::JointHandle joint_handle(joint_name, "position");
    if (rh_ptr->get_joint_handle(joint_handle) ==
      hardware_interface::hardware_interface_ret_t::ERROR)
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          logger_name_), "could not get handle for joint '" << joint_name << "'");
      return CallbackReturn::ERROR;
    }
    joint_state_handle_ =
      std::make_shared<hardware_interface::JointHandle>(std::move(joint_handle));
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name_), "could not lock pointer to robot_hardware");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SingleJointPositionController::update()
{
  auto joint_command = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_command || !(*joint_command)) {
    return controller_interface::return_type::SUCCESS;
  }

  const double current_position = joint_state_handle_->get_value();
  const double command_position = (*joint_command)->data;

  /// @todo enforce joint position limits?

  /// @todo error should be calculated according to the joint type, i.e.,
  /// revolute, continous or prismatic?
  const double position_error = command_position - current_position;

  const double command_velocity = pid_->computeCommand(position_error, pid_duration_);

  joint_cmd_handle_->set_value(command_velocity);

  return controller_interface::return_type::SUCCESS;
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::SingleJointPositionController, controller_interface::ControllerInterface)
