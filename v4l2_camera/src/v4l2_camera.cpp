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

#include "v4l2_camera/v4l2_camera.hpp"

#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <algorithm>

#include "v4l2_camera/fourcc.hpp"

#include "v4l2_camera/v4l2_camera_device.hpp"

#ifdef ENABLE_CUDA
#include <cuda.h>
#include <nppi_color_conversion.h>
#endif

using namespace std::chrono_literals;

namespace v4l2_camera
{
V4L2Camera::V4L2Camera(ros::NodeHandle node, ros::NodeHandle private_nh)
  : image_transport_(private_nh),
    node(node),
    private_nh(private_nh),
    canceled_{false}
{
  private_nh.getParam("publish_rate", publish_rate_);
  private_nh.getParam("video_device", device);
  private_nh.getParam("use_v4l2_buffer_timestamps", use_v4l2_buffer_timestamps);
  private_nh.getParam("timestamp_offset", timestamp_offset);
  private_nh.getParam("use_image_transport", use_image_transport_);

  if(std::abs(publish_rate_) < std::numeric_limits<double>::epsilon()){
    ROS_WARN("Invalid publish_rate = 0. Use default value -1 instead");
    publish_rate_ = -1.0;
  }
  // if(publish_rate_ > 0){
  //   const auto publish_period = ros::Duration(publish_rate_);
  //   image_pub_timer_ = node.createTimer(publish_period, &V4L2Camera::publishTimer, this);
  //   publish_next_frame_ = false;
  // }
  // else{
  //   publish_next_frame_ = true;
  // }
  if (use_image_transport_) {
    camera_transport_pub_ = image_transport_.advertiseCamera("image_raw", 10);
  } else {
    image_pub_ = node.advertise<sensor_msgs::Image>("image_raw", 10);
    info_pub_ = node.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
  }

  ros::Duration timestamp_offset_duration(0, timestamp_offset);

  camera_ = std::make_shared<V4l2CameraDevice>(device, use_v4l2_buffer_timestamps, timestamp_offset_duration);
  
  if (!camera_->open()) {
    return;
  }

  auto camera_info_url_ = "";
  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(private_nh, camera_->getCameraName(), camera_info_url_);
#ifdef ENABLE_CUDA
  src_dev_ = std::allocate_shared<GPUMemoryManager>(allocator_);
  dst_dev_ = std::allocate_shared<GPUMemoryManager>(allocator_);
#endif

  createParameters();

  // Start the camera
  if (!camera_->start()) {
    return;
  }

  // Start capture thread
  capture_thread_ = std::thread{
    [this]() -> void {
      while (ros::ok() && !canceled_.load()) {
        ROS_DEBUG("Capture...");
        auto img = camera_->capture();

        if (img == nullptr) {
          // Failed capturing image, assume it is temporarily and continue a bit later
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        if(publish_next_frame_ == false){
          continue;
        }

        auto stamp = img->header.stamp;
        if (img->encoding != output_encoding_) {
#ifdef ENABLE_CUDA
          img = convertOnGpu(*img);
#else
          img = convert(*img);
#endif
        }
        img->header.stamp = stamp;
        img->header.frame_id = camera_frame_id_;

        auto ci = std::make_unique<sensor_msgs::CameraInfo>(cinfo_->getCameraInfo());
        if (!checkCameraInfo(*img, *ci)) {
          *ci = sensor_msgs::CameraInfo{};
          ci->height = img->height;
          ci->width = img->width;
        }

        ci->header.stamp = stamp;
        ci->header.frame_id = camera_frame_id_;
        publish_next_frame_ = publish_rate_ < 0;

        if (use_image_transport_) {
          camera_transport_pub_.publish(*img, *ci);
        } else {
          image_pub_.publish(*img);
          info_pub_.publish(*ci);
        }
      }
    }
  };
}

void V4L2Camera::createParameters()
{
  // Node parameters
  ROS_INFO("Currently supported: 'rgb8', 'yuv422' or 'mono'");
  private_nh.param<std::string>("output_encoding",
                          output_encoding_,
                          std::string{"rgb8"});

  // Camera info parameters
  private_nh.param<std::string>("camera_info_url",
                        camera_info_url_,
                        "");


  if (private_nh.getParam("camera_info_url", camera_info_url_)) {
    if (cinfo_->validateURL(camera_info_url_)) {
      cinfo_->loadCameraInfo(camera_info_url_);
    } else {
      ROS_WARN("Invalid camera info URL: %s", camera_info_url_.c_str());
    }
  }
  ROS_INFO("Frame id inserted in published image");
  private_nh.param<std::string>("camera_frame_id",
                        camera_frame_id_,
                        "camera");
  // Format parameters
  // Pixel format
  auto const & image_formats = camera_->getImageFormats();
  auto pixel_format_constraints = std::ostringstream{};
  for (auto const & format : image_formats) {
    pixel_format_constraints <<
      "\"" << FourCC::toString(format.pixelFormat) << "\"" <<
      " (" << format.description << "), ";
  }
  auto str = pixel_format_constraints.str();
  str = str.substr(0, str.size() - 2);
  // pixel_format_descriptor.additional_constraints = str;

  ROS_INFO("Pixel format (FourCC)");
  private_nh.param<std::string>("pixel_format",
                        pixel_format_,
                        "YUYV");

  requestPixelFormat(pixel_format_);

  // Image size
  auto image_size = ImageSize{};

  // List available image sizes per format
  auto const & image_sizes = camera_->getImageSizes();
  auto image_sizes_constraints = std::ostringstream{};
  image_sizes_constraints << "Available image sizes:";

  for (auto const & format : image_formats) {
    image_sizes_constraints << "\n" << FourCC::toString(format.pixelFormat) << " (" <<
      format.description << ")";

    auto iter = image_sizes.find(format.pixelFormat);
    if (iter == image_sizes.end()) {
      ROS_ERROR_STREAM(
        "No sizes available to create parameter description for format: " << format.description);
      continue;
    }

    auto size_type = iter->second.first;
    auto & sizes = iter->second.second;
    switch (size_type) {
      case V4l2CameraDevice::ImageSizeType::DISCRETE:
        for (auto const & image_size : sizes) {
          image_sizes_constraints << "\n\t" << image_size.first << "x" << image_size.second;
        }
        break;
      case V4l2CameraDevice::ImageSizeType::STEPWISE:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        image_sizes_constraints << "\n\tstep:\t" << sizes[2].first << "x" << sizes[2].second;
        break;
      case V4l2CameraDevice::ImageSizeType::CONTINUOUS:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        break;
    }
  }

  ROS_INFO("Image width & height");
  private_nh.param<int>("image_size",image_size_width, 640);
  private_nh.param<int>("image_size",image_size_height, 480);
  image_size = {image_size_width, image_size_height};
  requestImageSize(image_size);

  // Time per frame
  if (camera_->timePerFrameSupported()) {
    auto tpf = camera_->getCurrentTimePerFrame();

    ROS_INFO("Desired period between successive frames in seconds");
    private_nh.param<int>("time_per_frame_first", time_per_frame_first, tpf.first);
    private_nh.param<int>("time_per_frame_second", time_per_frame_second, tpf.second);
    TimePerFrame time_per_frame = {static_cast<int64_t>(tpf.first), static_cast<int64_t>(tpf.second)};

    requestTimePerFrame(time_per_frame);
  }

  // Control parameters
  auto toParamName =
    [](std::string name) {
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      name.erase(std::remove(name.begin(), name.end(), ','), name.end());
      name.erase(std::remove(name.begin(), name.end(), '('), name.end());
      name.erase(std::remove(name.begin(), name.end(), ')'), name.end());
      std::replace(name.begin(), name.end(), ' ', '_');
      return name;
    };

  for (auto const & c : camera_->getControls()) {
    auto name = toParamName(c.name);
    switch (c.type) {
      case ControlType::INT:
        {
          auto current_value = camera_->getControlValue(c.id);
          if (current_value < c.minimum || c.maximum < current_value) {
            current_value = c.defaultValue;
          }
          ROS_INFO_STREAM(c.name);
          int32_t value;
          private_nh.param<int32_t>(name,
                                value,
                                current_value);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::BOOL:
        {
          ROS_INFO_STREAM(c.name);
          bool value;
          private_nh.param<bool>(name,
                                value,
                                camera_->getControlValue(c.id) != 0);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::MENU:
        {
          auto sstr = std::ostringstream{};
          for (auto const & o : c.menuItems) {
            sstr << o.first << " - " << o.second << ", ";
          }
          auto str = sstr.str();
          ROS_INFO_STREAM(c.name);
          int32_t value;
          private_nh.param<int32_t>(name,
                                value,
                                camera_->getControlValue(c.id));
          camera_->setControlValue(c.id, value);
          break;
        }
      default:
        ROS_WARN(
          "Control type not currently supported: %s, for control: %s",
          std::to_string(unsigned(c.type)).c_str(),
          c.name.c_str());
        continue;
    }
    control_name_to_id_[name] = c.id;
  }

  // Register callback for parameter value setting
  // on_set_parameters_callback_ = add_on_set_parameters_callback(
  //   [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
  //     auto result = rcl_interfaces::msg::SetParametersResult();
  //     result.successful = true;
  //     for (auto const & p : parameters) {
  //       result.successful &= handleParameter(p);
  //     }
  //     return result;
  //   });
}

bool V4L2Camera::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    ROS_ERROR("Invalid pixel format size: must be a 4 character code (FOURCC).");
    return false;
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code) {
    return true;
  }

  dataFormat.pixelFormat = code;
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() != 2) {
    ROS_WARN("Invalid image size; expected dimensions: 2, actual: %lu", size.size());
    return false;
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
    return true;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestTimePerFrame(TimePerFrame const & tpf)
{
  if (tpf.size() != 2) {
    ROS_WARN(
      "Invalid time per frame; expected dimensions: 2, actual: %lu", tpf.size());
    return false;
  }

  return camera_->requestTimePerFrame(std::make_pair(tpf[0], tpf[1]));
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

static void YUV2RGB(
  const unsigned char y, const unsigned char u, const unsigned char v, unsigned char * r,
  unsigned char * g, unsigned char * b)
{
  const int y2 = static_cast<int>(y);
  const int u2 = static_cast<int>(u) - 128;
  const int v2 = static_cast<int>(v) - 128;
  // std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  // std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

static void yuyv2rgb(unsigned char const * YUV, unsigned char * RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
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

sensor_msgs::ImagePtr V4L2Camera::convert(sensor_msgs::Image& img)
{
  // TODO(sander): temporary until cv_bridge and image_proc are available in ROS 2
  if (img.encoding == sensor_msgs::image_encodings::YUV422 &&
    output_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    auto outImg = boost::make_shared<sensor_msgs::Image>();
    outImg->width = img.width;
    outImg->height = img.height;
    outImg->step = img.width * 3;
    outImg->encoding = output_encoding_;
    outImg->data.resize(outImg->height * outImg->step);
    for (auto i = 0u; i < outImg->height; ++i) {
      yuyv2rgb(
        img.data.data() + i * img.step, outImg->data.data() + i * outImg->step,
        outImg->width);
    }
    return outImg;
  } else {
    ROS_WARN_ONCE(
      "Conversion not supported yet: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());
    return nullptr;
  }
}

#ifdef ENABLE_CUDA
void cudaErrorCheck(const cudaError_t e)
{
  if (e != cudaSuccess) {
    std::stringstream ss;
    ss << cudaGetErrorName(e) << " : " << cudaGetErrorString(e);
    throw std::runtime_error{ss.str()};
  }
}

sensor_msgs::ImagePtr V4L2Camera::convertOnGpu(sensor_msgs::Image& img)
{
  if ((img.encoding != sensor_msgs::image_encodings::YUV422 &&
      img.encoding != sensor_msgs::image_encodings::YUV422_YUY2) ||
      output_encoding_ != sensor_msgs::image_encodings::RGB8) {
    ROS_WARN_ONCE(
        "Conversion not supported yet: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());
    return nullptr;
  }

  auto outImg = boost::make_shared<sensor_msgs::Image>();
  outImg->width = img.width;
  outImg->height = img.height;
  outImg->step = img.width * 3;
  outImg->encoding = output_encoding_;
  outImg->data.resize(outImg->height * outImg->step);

  src_dev_->Allocate(img.width, img.height);
  dst_dev_->Allocate(outImg->width, outImg->height);

  unsigned int src_num_channel = static_cast<int>(img.step / img.width);  // No padded input is assumed
  cudaErrorCheck(cudaMemcpy2DAsync(static_cast<void*>(src_dev_->dev_ptr),
                                   src_dev_->step_bytes,
                                   static_cast<const void*>(img.data.data()),
                                   img.step,  // in byte. including padding
                                   img.width * src_num_channel * sizeof(Npp8u),  // in byte
                                   img.height,                 // in pixel
                                   cudaMemcpyHostToDevice));

  NppiSize roi = {static_cast<int>(img.width), static_cast<int>(img.height)};
  NppStatus res;
  if (img.encoding == sensor_msgs::image_encodings::YUV422_YUY2) {
    res = nppiYUV422ToRGB_8u_C2C3R(src_dev_->dev_ptr,
                                             src_dev_->step_bytes,
                                             dst_dev_->dev_ptr,
                                             dst_dev_->step_bytes,
                                             roi);
  } else {
    res = nppiCbYCr422ToRGB_8u_C2C3R(src_dev_->dev_ptr,
                                     src_dev_->step_bytes,
                                     dst_dev_->dev_ptr,
                                     dst_dev_->step_bytes,
                                     roi);
  }

  if (res != NPP_SUCCESS) {
    ROS_ERROR("NPP Error: %d", res);
  }

  cudaErrorCheck(cudaMemcpy2DAsync(static_cast<void*>(outImg->data.data()),
                                   outImg->step,
                                   static_cast<const void*>(dst_dev_->dev_ptr),
                                   dst_dev_->step_bytes,
                                   outImg->width * 3 * sizeof(Npp8u),  // in byte. exclude padding
                                   outImg->height,
                                   cudaMemcpyDeviceToHost));

  cudaErrorCheck(cudaDeviceSynchronize());

  return outImg;
}
#endif

void V4L2Camera::publishTimer()
{
  this->publish_next_frame_=true;
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::Image const & img,
  sensor_msgs::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}
}  // namespace v4l2_camera
