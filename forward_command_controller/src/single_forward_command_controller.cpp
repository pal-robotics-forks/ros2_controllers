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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "forward_command_controller/single_forward_command_controller.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

namespace
{
constexpr auto kSFCCLoggerName = "single forward command controller";
}

namespace forward_command_controller
{
using CallbackReturn = SingleForwardCommandController::CallbackReturn;

SingleForwardCommandController::SingleForwardCommandController()
: controller_interface::ControllerInterface(),
  joint_cmd_handle_(),
  rt_command_ptr_(nullptr),
  joint_command_subscriber_(nullptr),
  logger_name_(kSFCCLoggerName)
{}

CallbackReturn SingleForwardCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rclcpp::Parameter joint_param, interface_param;
  if (!lifecycle_node_->get_parameter("joint", joint_param)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "'joint' parameter not set");
    return CallbackReturn::ERROR;
  }
  if (!lifecycle_node_->get_parameter("interface_name", interface_param)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "'interface_name' parameter not set");
    return CallbackReturn::ERROR;
  }

  auto joint_name = joint_param.as_string();
  if (joint_name.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "'joint' is empty");
    return CallbackReturn::ERROR;
  }

  auto interface_name = interface_param.as_string();
  if (interface_name.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name_), "'interface_name' is empty");
    return CallbackReturn::ERROR;
  }

  if (auto rh_ptr = robot_hardware_.lock()) {
    const auto registered_joints = rh_ptr->get_registered_joint_names();

    // check joint is present
    if (std::find(
        registered_joints.cbegin(), registered_joints.cend(),
        joint_name) == registered_joints.cend())
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          logger_name_), "joint '" << joint_name << "' not registered");
      return CallbackReturn::ERROR;
    }

    // get joint handle
    hardware_interface::JointHandle joint_handle(joint_name, interface_name);
    if (rh_ptr->get_joint_handle(joint_handle) ==
      hardware_interface::hardware_interface_ret_t::ERROR)
    {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          logger_name_), "could not get handle for joint '" << joint_name << "'");
      return CallbackReturn::ERROR;
    }
    joint_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(std::move(joint_handle));

  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        logger_name_), "could not lock pointer to robot_hardware");
    return CallbackReturn::ERROR;
  }

  joint_command_subscriber_ = lifecycle_node_->create_subscription<CmdType>(
    "command", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      rt_command_ptr_.writeFromNonRT(msg);
    });

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      logger_name_), "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleForwardCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleForwardCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SingleForwardCommandController::update()
{
  auto joint_command = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_command || !(*joint_command)) {
    return controller_interface::return_type::SUCCESS;
  }

  joint_cmd_handle_->set_value((*joint_command)->data);

  return controller_interface::return_type::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::SingleForwardCommandController,
  controller_interface::ControllerInterface)
