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

#include "test_camera_sensor_broadcaster.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "camera_sensor_broadcaster/camera_sensor_broadcaster.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "sensor_msgs/msg/image.hpp"

using hardware_interface::LoanedStateInterface;

namespace
{
constexpr auto NODE_SUCCESS =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  return wait_set.wait().kind();
}

}  // namespace

void CameraSensorBroadcasterTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void CameraSensorBroadcasterTest::TearDownTestCase() { rclcpp::shutdown(); }

void CameraSensorBroadcasterTest::SetUp()
{
  // initialize controller
  camera_broadcaster_ = std::make_unique<FriendCameraSensorBroadcaster>();
}

void CameraSensorBroadcasterTest::TearDown() { camera_broadcaster_.reset(nullptr); }

void CameraSensorBroadcasterTest::SetUpCameraBroadcaster()
{
  const auto result = camera_broadcaster_->init("test_camera_sensor_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(camera_height_);
  state_ifs.emplace_back(camera_width_);
  // state_ifs.emplace_back(camera_orientation_x_);
  // state_ifs.emplace_back(camera_orientation_y_);
  // state_ifs.emplace_back(camera_orientation_z_);
  // state_ifs.emplace_back(camera_orientation_w_);
  // state_ifs.emplace_back(camera_angular_velocity_x_);
  // state_ifs.emplace_back(camera_angular_velocity_y_);
  // state_ifs.emplace_back(camera_angular_velocity_z_);
  // state_ifs.emplace_back(camera_linear_acceleration_x_);
  // state_ifs.emplace_back(camera_linear_acceleration_y_);
  // state_ifs.emplace_back(camera_linear_acceleration_z_);

  camera_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

void CameraSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::Image & camera_msg)
{
  // create a new subscriber
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Image::SharedPtr) {};
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Image>(
    "/test_camera_sensor_broadcaster/camera", 2, subs_callback); //10, subs_callback);

  // call update to publish the test value
  ASSERT_EQ(camera_broadcaster_->update(), controller_interface::return_type::OK);

  // wait for message to be passed
  ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

  // take message from subscription
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(camera_msg, msg_info));
}

TEST_F(CameraSensorBroadcasterTest, SensorName_InterfaceNames_NotSet)
{
  SetUpCameraBroadcaster();

  // configure failed
  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(CameraSensorBroadcasterTest, SensorName_FrameId_NotSet)
{
  SetUpCameraBroadcaster();

  // set the 'interface_names'
  camera_broadcaster_->get_node()->set_parameter(
    {"interface_names.height", "camera_sensor/height"});
  camera_broadcaster_->get_node()->set_parameter(
    {"interface_names.width", "camera_sensor/width"});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(CameraSensorBroadcasterTest, InterfaceNames_FrameId_NotSet)
{
  SetUpCameraBroadcaster();

  // set the 'sensor_name'
  camera_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // configure failed, 'frame_id' parameter not set
  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(CameraSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpCameraBroadcaster();

  // set the 'sensor_name'
  camera_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // set the 'frame_id'
  camera_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure passed
  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(CameraSensorBroadcasterTest, SensorName_Activate_Success)
{
  SetUpCameraBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  camera_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  camera_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure and activate success
  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(camera_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(CameraSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpCameraBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  camera_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  camera_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(camera_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(camera_broadcaster_->update(), controller_interface::return_type::OK);
}

TEST_F(CameraSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpCameraBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  camera_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  camera_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(camera_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(camera_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::Image camera_msg;
  subscribe_and_get_message(camera_msg);

  ASSERT_EQ(camera_msg.header.frame_id, frame_id_);
  ASSERT_EQ(camera_msg.height, sensor_values_[0]);
  ASSERT_EQ(camera_msg.width, sensor_values_[1]);
  // ASSERT_EQ(camera_msg.orientation.x, sensor_values_[0]);
  // ASSERT_EQ(camera_msg.orientation.y, sensor_values_[1]);
  // ASSERT_EQ(camera_msg.orientation.z, sensor_values_[2]);
  // ASSERT_EQ(camera_msg.orientation.w, sensor_values_[3]);
  // ASSERT_EQ(camera_msg.angular_velocity.x, sensor_values_[4]);
  // ASSERT_EQ(camera_msg.angular_velocity.y, sensor_values_[5]);
  // ASSERT_EQ(camera_msg.angular_velocity.z, sensor_values_[6]);
  // ASSERT_EQ(camera_msg.linear_acceleration.x, sensor_values_[7]);
  // ASSERT_EQ(camera_msg.linear_acceleration.y, sensor_values_[8]);
  // ASSERT_EQ(camera_msg.linear_acceleration.z, sensor_values_[9]);
}
