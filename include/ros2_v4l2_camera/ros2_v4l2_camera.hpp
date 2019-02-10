// Copyright 2019 Bold Hearts
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

#ifndef ROS2_V4L2_CAMERA__ROS2_V4L2_CAMERA_HPP_
#define ROS2_V4L2_CAMERA__ROS2_V4L2_CAMERA_HPP_

#include "ros2_v4l2_camera/visibility_control.h"

#include "ros2_v4l2_camera/v4l2_camera.hpp"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>

namespace ros2_v4l2_camera
{

class Ros2V4L2Camera : public rclcpp::Node
{
public:
  Ros2V4L2Camera();

  virtual ~Ros2V4L2Camera();

private:
  std::shared_ptr<V4l2Camera> camera_;
  image_transport::Publisher image_pub_;

  rclcpp::TimerBase::SharedPtr capture_timer_;

  std::string output_encoding_;

  sensor_msgs::msg::Image convert(sensor_msgs::msg::Image const& img) const;
};

}  // namespace ros2_v4l2_camera

#endif  // ROS2_V4L2_CAMERA__ROS2_V4L2_CAMERA_HPP_
