// Copyright 2021 PAL Robotics SL.
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

/*
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#include "range_sensor_detected_point_broadcaster/range_sensor_detected_point_broadcaster.hpp"

#include <memory>
#include <string>
#include "ros2_interfaces/msg/range_sensor_detected_point.hpp"

namespace range_sensor_detected_point_broadcaster
{
controller_interface::CallbackReturn RangeSensorDetectedPointBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RangeSensorDetectedPointBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  range_sensor_detected_point_ = std::make_unique<semantic_components::RangeSensorDetectedPoint>(
    semantic_components::RangeSensorDetectedPoint(params_.sensor_name));
  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ =
    get_node()->create_publisher<ros2_interfaces::msg::RangeSensorDetectedPoint>("~/prox_sensor_detected_point", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->unlock();

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RangeSensorDetectedPointBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration RangeSensorDetectedPointBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = range_sensor_detected_point_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn RangeSensorDetectedPointBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  range_sensor_detected_point_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RangeSensorDetectedPointBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  range_sensor_detected_point_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RangeSensorDetectedPointBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    // realtime_publisher_->msg_.header.stamp = get_node()->now();
    range_sensor_detected_point_->get_values_as_message(realtime_publisher_->msg_);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace range_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  range_sensor_detected_point_broadcaster::RangeSensorDetectedPointBroadcaster, controller_interface::ControllerInterface)
