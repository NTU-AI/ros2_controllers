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

#ifndef TIME_SENSOR_BROADCASTER__TIME_SENSOR_BROADCASTER_HPP_
#define TIME_SENSOR_BROADCASTER__TIME_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "time_sensor_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "time_sensor.hpp"
//#include "sensor_msgs/msg/time_reference.hpp"
#include "std_msgs/msg/float32.hpp"

namespace time_sensor_broadcaster
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TimeSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  TIME_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  TIME_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TIME_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TIME_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  TIME_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TIME_SENSOR_BROADCASTER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TIME_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::string sensor_name_;
  std::string frame_id_;

  std::unique_ptr<semantic_components::TimeSensor> time_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float32>;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace time_sensor_broadcaster

#endif  // TIME_SENSOR_BROADCASTER__TIME_SENSOR_BROADCASTER_HPP_
