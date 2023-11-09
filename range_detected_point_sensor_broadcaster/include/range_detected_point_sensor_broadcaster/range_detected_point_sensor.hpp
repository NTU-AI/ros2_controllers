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

#ifndef SEMANTIC_COMPONENTS__RANGE_DETECTED_POINT_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__RANGE_DETECTED_POINT_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "ros2_interfaces/msg/range_detected_point_sensor.hpp"

namespace semantic_components
{
class RangeDetectedPointSensor : public SemanticComponentInterface<ros2_interfaces::msg::RangeDetectedPointSensor>
{
public:
  explicit RangeDetectedPointSensor(const std::string & name) : SemanticComponentInterface(name, 4)
  {
    interface_names_.emplace_back(name_ + "/" + "range");
    interface_names_.emplace_back(name_ + "/" + "detected_point");
    interface_names_.emplace_back(name_ + "/" + "detected_handle");
    interface_names_.emplace_back(name_ + "/" + "normal_vector");
    
    range_ = std::numeric_limits<float>::quiet_NaN();

    detected_point_.resize(3);
    detected_point_[0] = std::numeric_limits<float>::quiet_NaN();
    detected_point_[1] = std::numeric_limits<float>::quiet_NaN();
    detected_point_[2] = std::numeric_limits<float>::quiet_NaN();

    handle_ = std::numeric_limits<int>::quiet_NaN();

    normal_vector_.resize(3);
    normal_vector_[0] = std::numeric_limits<float>::quiet_NaN();
    normal_vector_[1] = std::numeric_limits<float>::quiet_NaN();
    normal_vector_[2] = std::numeric_limits<float>::quiet_NaN();
  }

  virtual ~RangeDetectedPointSensor() = default;

  float get_range()
  {
    size_t interface_offset = 0;
    range_ = state_interfaces_[interface_offset].get().get_value();
    return range_;
  }

  std::vector<float> get_detected_point(){
    size_t interface_offset = 1;
    auto arrayData = state_interfaces_[interface_offset].get().get_array_value();
    detected_point_.assign(arrayData.begin(), arrayData.end());
    return detected_point_;
  }

  int get_handle()
  {
    size_t interface_offset = 2;
    handle_ = state_interfaces_[interface_offset].get().get_int_value();
    return handle_;
  }

  std::vector<float> get_normal_vector(){
    size_t interface_offset = 3;
    auto arrayData = state_interfaces_[interface_offset].get().get_array_value();
    normal_vector_.assign(arrayData.begin(), arrayData.end());
    return normal_vector_;
  }

  bool get_values_as_message(ros2_interfaces::msg::RangeDetectedPointSensor & message)
  {
    get_range();
    get_detected_point();
    get_handle();
    get_normal_vector();

    // update the message values
    message.range = range_;
    
    message.detected_point.x = detected_point_[0];
    message.detected_point.y = detected_point_[1];
    message.detected_point.z = detected_point_[2];
    
    message.detected_handle = handle_;

    message.normal_vector.x = normal_vector_[0];
    message.normal_vector.y = normal_vector_[1];
    message.normal_vector.z = normal_vector_[2];

    return true;
  }

protected:
  // Order is: radiation_type, field_of_view, min_range, max_range, range
  float range_;
  std::vector<float> detected_point_;
  int handle_;
  std::vector<float> normal_vector_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_