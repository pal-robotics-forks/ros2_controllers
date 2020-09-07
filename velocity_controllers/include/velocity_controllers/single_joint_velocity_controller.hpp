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

#ifndef VELOCITY_CONTROLLERS__SINGLE_JOINT_VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROLLERS__SINGLE_JOINT_VELOCITY_CONTROLLER_HPP_

#include <memory>

#include "forward_command_controller/single_forward_command_controller.hpp"
#include "velocity_controllers/visibility_control.h"

namespace velocity_controllers
{

/**
 * \brief Forward command controller for a velocity controlled joint (linear or angular).
 *
 * This class forwards the commanded velocity down to a joint.
 *
 * \param joint Name of the joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64) : The velocity command to apply.
 */
class SingleJointVelocityController : public forward_command_controller::
  SingleForwardCommandController
{
public:
  VELOCITY_CONTROLLERS_PUBLIC
  SingleJointVelocityController();

  VELOCITY_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  VELOCITY_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
};

}  // namespace velocity_controllers

#endif  // VELOCITY_CONTROLLERS__SINGLE_JOINT_VELOCITY_CONTROLLER_HPP_
