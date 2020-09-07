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

#include <memory>

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
  pid_(nullptr)
{
  logger_name_ = kSJPCLoggerName;
}

CallbackReturn SingleJointPositionController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  pid_ = std::make_shared<control_toolbox::PidROS>(
    lifecycle_node_->get_node_base_interface(),
    lifecycle_node_->get_node_logging_interface(),
    lifecycle_node_->get_node_parameters_interface(),
    lifecycle_node_->get_node_topics_interface(), "pid");

  if (!pid_->initPid()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "failed to initialize PID");
    return CallbackReturn::ERROR;
  }

  return SingleJointVelocityController::on_configure(previous_state);
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::SingleJointPositionController, controller_interface::ControllerInterface)
