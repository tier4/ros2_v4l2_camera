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

#ifndef ROS2_V4L2_CAMERA__V4L2_CAMERA_HPP_
#define ROS2_V4L2_CAMERA__V4L2_CAMERA_HPP_

#include <linux/videodev2.h>

#include <sensor_msgs/msg/image.hpp>
#include <map>

namespace ros2_v4l2_camera
{

/** Camera device using Video4Linux2
 */
class V4l2Camera {
public:
  V4l2Camera(std::string device);

  bool open();
  void start();
  void stop();

  sensor_msgs::msg::Image capture();

private:
  /** Image format description
   *
   * Describes what image data means
   */
  struct ImageFormat
  {
    ImageFormat(v4l2_fmtdesc const& fd)
    : index(fd.index),
      type(fd.type),
      flags(fd.flags),
      description((const char*)fd.description),
      pixelFormat(fd.pixelformat)
    {}
    
    /// Number of the format in the enumeration, set by the application
    unsigned index;
    
    /// Type of the data stream, set by the application, probably to V4L2_BUF_TYPE_VIDEO_CAPTURE
    unsigned type;
    
    /// Image format description flags. Options: V4L2_FMT_FLAG_COMPRESSED and/or V4L2_FMT_FLAG_EMULATED
    unsigned flags;
    
    /// Human readable description of the format
    std::string description;
    
    /// The image format identifier as computed by the v4l2_fourcc() macro
    unsigned pixelFormat;
  };
  
  /** Format and layout of an image in memory
   * 
   * Describes how data is structured
   */
  struct PixelFormat
  {
    PixelFormat()
    {}
    
    PixelFormat(v4l2_pix_format const& pf)
    : width (pf.width),
      height(pf.height),
      pixelFormat(pf.pixelformat),
      bytesPerLine(pf.bytesperline),
      imageByteSize(pf.sizeimage)
    {}
    
    /// Image width in pixels
    unsigned width;
    
    /// Image height in pixels
    unsigned height;
    
    /// The pixel format or type of compression, set by the application
    unsigned pixelFormat;
    
    /// Distance in bytes between the leftmost pixels in two adjacent lines
    unsigned bytesPerLine;
    
    /// Size in bytes of the buffer to hold a complete image, set by the driver
    unsigned imageByteSize;
    
    /// Human readable description of the format
    std::string pixelFormatString() const
    {
      char chars[5];
      for (unsigned i = 0; i < 4; ++i)
        chars[i] = ((pixelFormat >> (i * 8)) & 0xFF);
      chars[4] = 0;
      return std::string(chars);
    }
  };

  /// Type of camera control
  enum class ControlType : unsigned
  {
    INT        = 1,
    BOOL       = 2,
    MENU       = 3,
    BUTTON     = 4,
    INT64      = 5,
    CTRL_CLASS = 6,
    STRING     = 7,
    BITMASK    = 8
  };

  struct Control
  {
    /// Identifies the control, set by the application
    unsigned id;
    
    /// Human readable name
    std::string name;
    
    /// Type of control
    ControlType type;
        
    /// Minimum value, inclusive
    int minimum;
    
    /// Maximum value, inclusive
    int maximum;
    
    /// The default value of of an integer, boolean, bitmask, menu or integer menu control
    int defaultValue;
    
    /// Menu item names by index. Empty if this is not a menu control
    std::map<int,std::string> menuItems;
  };

  std::string device_;
  int fd_;

  v4l2_capability capabilities_;
  std::vector<ImageFormat> image_formats_;
  std::vector<Control> controls_;
  
  PixelFormat cur_data_format_;
  
  // Requests and stores all formats available for this camera
  void listImageFormats();

  // Requests and stores all controls available for this camera
  void listControls();
};

}

#endif

