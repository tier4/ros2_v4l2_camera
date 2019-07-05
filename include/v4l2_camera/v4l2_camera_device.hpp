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

#ifndef V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
#define V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_

#include <sensor_msgs/msg/image.hpp>

#include <map>
#include <string>
#include <vector>

#include "v4l2_camera/control.hpp"
#include "v4l2_camera/image_format.hpp"
#include "v4l2_camera/pixel_format.hpp"

namespace v4l2_camera
{

/** Camera device using Video4Linux2
 */
class V4l2CameraDevice
{
public:
  explicit V4l2CameraDevice(std::string device);

  bool open();
  bool start();
  bool stop();

  auto const & getControls() const {return controls_;}
  int32_t getControlValue(uint32_t id);
  bool setControlValue(uint32_t id, int32_t value);

  auto const & getCurrentDataFormat() const {return cur_data_format_;}
  bool requestDataFormat(PixelFormat const & format);

  std::string getCameraName();

  sensor_msgs::msg::Image::UniquePtr capture();

private:
  /// Image buffer
  struct Buffer
  {
    unsigned index;
    unsigned char * start;
    size_t length;
  };

  std::string device_;
  int fd_;

  v4l2_capability capabilities_;
  std::vector<ImageFormat> image_formats_;
  std::vector<Control> controls_;

  PixelFormat cur_data_format_;

  std::vector<Buffer> buffers_;

  // Requests and stores all formats available for this camera
  void listImageFormats();

  // Requests and stores all controls available for this camera
  void listControls();

  // Set up memory mapping to buffers
  bool initMemoryMapping();
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
