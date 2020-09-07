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

#include "velocity_controllers/single_joint_velocity_controller.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace
{
constexpr auto kSJPCLoggerName = "single joint velocity controller";
}

namespace velocity_controllers
{
using CallbackReturn = SingleJointVelocityController::CallbackReturn;

SingleJointVelocityController::SingleJointVelocityController()
: forward_command_controller::SingleForwardCommandController()
{
  logger_name_ = kSJPCLoggerName;
}

CallbackReturn SingleJointVelocityController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  rclcpp::Parameter interface_param;
  if (!lifecycle_node_->get_parameter("interface_name", interface_param)) {
    lifecycle_node_->declare_parameter("interface_name", "velocity_command");
  } else {
    if (interface_param.as_string() != "velocity_command") {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          kSJPCLoggerName), "'interface_name' already set with an invalid value");
      return CallbackReturn::ERROR;
    }
  }
  return SingleForwardCommandController::on_configure(previous_state);
}

CallbackReturn SingleJointVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = SingleForwardCommandController::on_deactivate(previous_state);

  // stop the joint
  joint_cmd_handle_->set_value(0.0);

  return ret;
}

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::SingleJointVelocityController, controller_interface::ControllerInterface)
