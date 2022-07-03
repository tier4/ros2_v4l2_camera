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
#include <utility>
#include <tuple>
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

  // Types used to describe available image sizes
  enum class ImageSizeType
  {
    DISCRETE,
    STEPWISE,
    CONTINUOUS
  };

  // Sizes are in width/height order
  using ImageSizesVector = std::vector<std::pair<uint16_t, uint16_t>>;
  using ImageSizesDescription = std::pair<ImageSizeType, ImageSizesVector>;
  // Interval in seconds described by ration of numerator (first) and denominator (second)
  using FrameIntervalsVector = std::vector<std::pair<uint32_t, uint32_t>>;

  auto const & getImageFormats() const {return image_formats_;}
  auto const & getImageSizes() const {return image_sizes_;}
  auto const & getFrameIntervals() const {return frame_intervals_;}
  auto const & getCurrentDataFormat() const {return cur_data_format_;}
  bool requestDataFormat(PixelFormat const & format);

  bool timePerFrameSupported() const
  {
    return (capture_parm_.capability & V4L2_CAP_TIMEPERFRAME) != 0;
  }

  auto getCurrentTimePerFrame() const
  {
    return std::make_pair(
      capture_parm_.timeperframe.numerator,
      capture_parm_.timeperframe.denominator);
  }

  bool requestTimePerFrame(std::pair<uint32_t, uint32_t> tpf);

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
  v4l2_captureparm capture_parm_;

  std::vector<ImageFormat> image_formats_;
  // Keyed by ImageFormat::pixelFormat
  std::map<unsigned, ImageSizesDescription> image_sizes_;
  // Keyed by ImageFormat::pixelFormat, width and height
  std::map<std::tuple<unsigned, uint16_t, uint16_t>, FrameIntervalsVector> frame_intervals_;

  std::vector<Control> controls_;

  PixelFormat cur_data_format_;

  std::vector<Buffer> buffers_;

  void getCaptureParameters();

  // Requests and stores all formats available for this camera
  void listImageFormats();

  // Requests and stores all frame sizes available for this camera
  void listImageSizes();

  ImageSizesDescription listDiscreteImageSizes(v4l2_frmsizeenum frm_size_enum);
  ImageSizesDescription listStepwiseImageSizes(v4l2_frmsizeenum frm_size_enum);
  ImageSizesDescription listContinuousImageSizes(v4l2_frmsizeenum frm_size_enum);

  // Requests and stores all frame intervals
  void listFrameIntervals();

  // Requests and stores all controls available for this camera
  void listControls();

  // Set up memory mapping to buffers
  bool initMemoryMapping();
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__V4L2_CAMERA_DEVICE_HPP_
