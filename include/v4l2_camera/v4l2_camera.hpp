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

#ifndef V4L2_CAMERA__V4L2_CAMERA_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_HPP_

#include "v4l2_camera/visibility_control.h"

#include "v4l2_camera/v4l2_camera_device.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

namespace v4l2_camera
{

class V4L2Camera : public rclcpp::Node
{
public:
  V4L2Camera();

  virtual ~V4L2Camera();

private:
  std::shared_ptr<V4l2CameraDevice> camera_;
  image_transport::CameraPublisher camera_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  rclcpp::TimerBase::SharedPtr capture_timer_;

  std::string output_encoding_;

  std::map<std::string, int32_t> control_name_to_id_;

  void createParameters();
  bool handleParameter(rclcpp::Parameter const & param);

  bool requestImageSize(std::vector<int64_t> const & size);

  sensor_msgs::msg::Image convert(sensor_msgs::msg::Image const & img) const;
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__ROS2_V4L2_CAMERA_HPP_
