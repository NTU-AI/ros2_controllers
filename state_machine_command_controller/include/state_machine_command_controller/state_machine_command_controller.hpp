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

#ifndef STATE_MACHINE_COMMAND_CONTROLLER__STATE_MACHINE_COMMAND_CONTROLLER_HPP_
#define STATE_MACHINE_COMMAND_CONTROLLER__STATE_MACHINE_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "state_machine_command_controller/state_machine_controllers_base.hpp"
#include "state_machine_command_controller/visibility_control.h"
#include "state_machine_command_controller_parameters.hpp"

namespace state_machine_command_controller
{
/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints on the specified interface.
 *
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class StateMachineCommandController : public StateMachineControllersBase
{
public:
  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  StateMachineCommandController();

protected:
  void declare_parameters() override;
  controller_interface::CallbackReturn read_parameters() override;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace state_machine_command_controller

#endif  // STATE_MACHINE_COMMAND_CONTROLLER__STATE_MACHINE_COMMAND_CONTROLLER_HPP_