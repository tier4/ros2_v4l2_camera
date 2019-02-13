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

#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace ros2_v4l2_camera
{

Ros2V4L2Camera::Ros2V4L2Camera()
: rclcpp::Node{"ros2_v4l2_camera"}
{
  // Prepare camera
  camera_ = std::make_shared<V4l2Camera>("/dev/video0");
  if (!camera_->open())
    return;

  auto dataFormat = camera_->getCurrentDataFormat();
  
  // Read parameters
  get_parameter_or("output_encoding", output_encoding_, std::string{"rgb8"});
  get_parameter("width", dataFormat.width);
  get_parameter("height", dataFormat.height);
  camera_->requestDataFormat(dataFormat);
  
  if (!camera_->start())
    return;

  // Prepare publisher
  image_pub_ = image_transport::create_publisher(this, "/image_raw", rmw_qos_profile_sensor_data);

  
  // Start capture timer
  capture_timer_ = create_wall_timer(
    33ms, [=]() -> void {
            RCLCPP_DEBUG(get_logger(), "Capture...");
            auto img = camera_->capture();
            if (img.encoding != output_encoding_)
              img = convert(img);
            image_pub_.publish(img);
          });
}

Ros2V4L2Camera::~Ros2V4L2Camera()
{
}

static unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  val = val < 0 ? 0 : val;
  return val > 255 ? 255 : val;
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void YUV2RGB(const unsigned char y, const unsigned char u, const unsigned char v, unsigned char* r,
                    unsigned char* g, unsigned char* b)
{
  const int y2 = (int)y;
  const int u2 = (int)u - 128;
  const int v2 = (int)v - 128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

static void yuyv2rgb(unsigned char const *YUV, unsigned char *RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
  {
    y0 = YUV[i + 0];
    u = YUV[i + 1];
    y1 = YUV[i + 2];
    v = YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

sensor_msgs::msg::Image Ros2V4L2Camera::convert(sensor_msgs::msg::Image const& img) const
{
  RCLCPP_DEBUG(get_logger(),
    std::string{"Coverting: "} + img.encoding + " -> " + output_encoding_);
  
  // TODO: temporary until cv_bridge and image_proc are available in ROS 2
  if (img.encoding == sensor_msgs::image_encodings::YUV422
    && output_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    auto outImg = sensor_msgs::msg::Image{};
    outImg.width = img.width;
    outImg.height = img.height;
    outImg.step = img.width * 3;
    outImg.encoding = output_encoding_;
    outImg.data.resize(outImg.height * outImg.step);
    for (auto i = 0u; i < outImg.height; ++i)
      yuyv2rgb(img.data.data() + i * img.step, outImg.data.data() + i * outImg.step, outImg.width);
    return outImg;
  }
  else
  {
    RCLCPP_WARN_ONCE(get_logger(),
      std::string{"Conversion not supported yet: "} + img.encoding + " -> " + output_encoding_);
    return img;
  }
}

}  // namespace ros2_v4l2_camera
