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

#include "ros2_v4l2_camera/ros2_v4l2_camera.hpp"

using namespace std::chrono_literals;

namespace ros2_v4l2_camera
{

Ros2V4L2Camera::Ros2V4L2Camera()
: rclcpp::Node{"ros2_v4l2_camera"}
{
  camera_ = std::make_shared<V4l2Camera>("/dev/video0");
  if (!camera_->open())
    return;
  
  image_pub_ = image_transport::create_publisher(this, "/image_raw");

  capture_timer_ = create_wall_timer(
    33ms, [=]() -> void {
            RCLCPP_INFO(get_logger(), "Capture...");
            auto img = camera_->capture();
            image_pub_.publish(img);
          });
}

Ros2V4L2Camera::~Ros2V4L2Camera()
{
}

}  // namespace ros2_v4l2_camera
