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

#include "state_machine_command_controller/state_machine_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace state_machine_command_controller
{
StateMachineCommandController::StateMachineCommandController() : StateMachineControllersBase() {}

void StateMachineCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn StateMachineCommandController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.actuator_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'actuator_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interface_types_.push_back(params_.actuator_name + "/" + params_.interface_name);

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace state_machine_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  state_machine_command_controller::StateMachineCommandController, controller_interface::ControllerInterface)
