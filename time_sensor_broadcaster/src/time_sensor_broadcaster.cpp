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

#include "time_sensor_broadcaster/time_sensor_broadcaster.hpp"

#include <memory>
#include <string>

namespace time_sensor_broadcaster
{
controller_interface::return_type TimeSensorBroadcaster::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("frame_id", "");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn TimeSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = node_->get_parameter("sensor_name").as_string();
  if (sensor_name_.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "'sensor_name' parameter has to be specified.");
    return CallbackReturn::ERROR;
  }

  frame_id_ = node_->get_parameter("frame_id").as_string();
  if (frame_id_.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "'frame_id' parameter has to be provided.");
    return CallbackReturn::ERROR;
  }

  time_sensor_ =
    std::make_unique<semantic_components::TimeSensor>(semantic_components::TimeSensor(sensor_name_));
  try
  {
    // register ft sensor data publisher
    sensor_state_publisher_ =
      node_->create_publisher<std_msgs::msg::Float32>("~/time", rclcpp::SystemDefaultsQoS());
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
  //realtime_publisher_->msg_.header.frame_id = frame_id_;
  realtime_publisher_->unlock();

  RCLCPP_DEBUG(node_->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TimeSensorBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration TimeSensorBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = time_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

CallbackReturn TimeSensorBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  time_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TimeSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  time_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TimeSensorBroadcaster::update()
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    //realtime_publisher_->msg_.header.stamp = node_->now();
    time_sensor_->get_values_as_message(realtime_publisher_->msg_);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace time_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  time_sensor_broadcaster::TimeSensorBroadcaster, controller_interface::ControllerInterface)
