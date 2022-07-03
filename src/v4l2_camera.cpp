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

#include <sensor_msgs/image_encodings.hpp>

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <algorithm>

#include "v4l2_camera/fourcc.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace v4l2_camera
{

V4L2Camera::V4L2Camera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"v4l2_camera", options},
  canceled_{false}
{
  // Prepare publisher
  // This should happen before registering on_set_parameters_callback,
  // else transport plugins will fail to declare their parameters
  if (options.use_intra_process_comms()) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  } else {
    camera_transport_pub_ = image_transport::create_camera_publisher(this, "image_raw");
  }

  // Prepare camera
  auto device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  device_descriptor.description = "Path to video device";
  device_descriptor.read_only = true;
  auto device = declare_parameter<std::string>("video_device", "/dev/video0", device_descriptor);
  camera_ = std::make_shared<V4l2CameraDevice>(device);

  if (!camera_->open()) {
    return;
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());

  // Read parameters and set up callback
  createParameters();

  // Start the camera
  if (!camera_->start()) {
    return;
  }

  // Start capture thread
  capture_thread_ = std::thread{
    [this]() -> void {
      while (rclcpp::ok() && !canceled_.load()) {
        RCLCPP_DEBUG(get_logger(), "Capture...");
        auto img = camera_->capture();
        if (img == nullptr) {
          // Failed capturing image, assume it is temporarily and continue a bit later
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }

        auto stamp = now();
        if (img->encoding != output_encoding_) {
          img = convert(*img);
        }
        img->header.stamp = stamp;
        img->header.frame_id = camera_frame_id_;

        auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        if (!checkCameraInfo(*img, *ci)) {
          *ci = sensor_msgs::msg::CameraInfo{};
          ci->height = img->height;
          ci->width = img->width;
        }

        ci->header.stamp = stamp;

        if (get_node_options().use_intra_process_comms()) {
          RCLCPP_DEBUG_STREAM(get_logger(), "Image message address [PUBLISH]:\t" << img.get());
          image_pub_->publish(std::move(img));
          info_pub_->publish(std::move(ci));
        } else {
          camera_transport_pub_.publish(*img, *ci);
        }
      }
    }
  };
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void V4L2Camera::createParameters()
{
  // Node parameters
  auto output_encoding_description = rcl_interfaces::msg::ParameterDescriptor{};
  output_encoding_description.description = "ROS image encoding to use for the output image";
  output_encoding_description.additional_constraints =
    "Currently supported: 'rgb8', 'yuv422' or 'mono'";
  output_encoding_ = declare_parameter(
    "output_encoding", std::string{"rgb8"},
    output_encoding_description);

  // Camera info parameters
  auto camera_info_url = std::string{};
  if (get_parameter("camera_info_url", camera_info_url)) {
    if (cinfo_->validateURL(camera_info_url)) {
      cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  auto camera_frame_id_description = rcl_interfaces::msg::ParameterDescriptor{};
  camera_frame_id_description.description = "Frame id inserted in published image";
  camera_frame_id_description.read_only = true;
  camera_frame_id_ = declare_parameter<std::string>(
    "camera_frame_id", "camera",
    camera_frame_id_description);

  // Format parameters
  // Pixel format
  auto const & image_formats = camera_->getImageFormats();
  auto pixel_format_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  pixel_format_descriptor.name = "pixel_format";
  pixel_format_descriptor.description = "Pixel format (FourCC)";
  auto pixel_format_constraints = std::ostringstream{};
  for (auto const & format : image_formats) {
    pixel_format_constraints <<
      "\"" << FourCC::toString(format.pixelFormat) << "\"" <<
      " (" << format.description << "), ";
  }
  auto str = pixel_format_constraints.str();
  str = str.substr(0, str.size() - 2);
  pixel_format_descriptor.additional_constraints = str;
  auto pixel_format =
    declare_parameter<std::string>("pixel_format", "YUYV", pixel_format_descriptor);
  requestPixelFormat(pixel_format);

  // Image size
  auto image_size = ImageSize{};
  auto image_size_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  image_size_descriptor.name = "image_size";
  image_size_descriptor.description = "Image width & height";

  // List available image sizes per format
  auto const & image_sizes = camera_->getImageSizes();
  auto image_sizes_constraints = std::ostringstream{};
  image_sizes_constraints << "Available image sizes:";

  for (auto const & format : image_formats) {
    image_sizes_constraints << "\n" << FourCC::toString(format.pixelFormat) << " (" <<
      format.description << ")";

    auto iter = image_sizes.find(format.pixelFormat);
    if (iter == image_sizes.end()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
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

  image_size_descriptor.additional_constraints = image_sizes_constraints.str();
  image_size = declare_parameter<ImageSize>("image_size", {640, 480}, image_size_descriptor);
  requestImageSize(image_size);

  // Time per frame
  if (camera_->timePerFrameSupported()) {
    auto tpf = camera_->getCurrentTimePerFrame();
    auto time_per_frame_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    time_per_frame_descriptor.name = "time_per_frame";
    time_per_frame_descriptor.description = "Desired period between successive frames in seconds";
    time_per_frame_descriptor.additional_constraints =
      "Length 2 array, with numerator and denominator";
    auto time_per_frame = declare_parameter<TimePerFrame>(
      "time_per_frame", {tpf.first, tpf.second},
      time_per_frame_descriptor);
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
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.name = name;
    descriptor.description = c.name;
    switch (c.type) {
      case ControlType::INT:
        {
          auto current_value = camera_->getControlValue(c.id);
          auto range = rcl_interfaces::msg::IntegerRange{};
          range.from_value = c.minimum;
          range.to_value = c.maximum;
          descriptor.integer_range.push_back(range);
          auto value = declare_parameter<int64_t>(name, current_value, descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::BOOL:
        {
          auto value =
            declare_parameter<bool>(name, camera_->getControlValue(c.id) != 0, descriptor);
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
          descriptor.additional_constraints = str.substr(0, str.size() - 2);
          auto value = declare_parameter<int64_t>(name, camera_->getControlValue(c.id), descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control type not currently supported: %s, for control: %s",
          std::to_string(unsigned(c.type)).c_str(),
          c.name.c_str());
        continue;
    }
    control_name_to_id_[name] = c.id;
  }

  // Register callback for parameter value setting
  on_set_parameters_callback_ = add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const & p : parameters) {
        result.successful &= handleParameter(p);
      }
      return result;
    });
}

bool V4L2Camera::handleParameter(rclcpp::Parameter const & param)
{
  auto name = std::string{param.get_name()};
  if (control_name_to_id_.find(name) != control_name_to_id_.end()) {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return camera_->setControlValue(control_name_to_id_[name], param.as_bool());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return camera_->setControlValue(control_name_to_id_[name], param.as_int());
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control parameter type not currently supported: %s, for parameter: %s",
          std::to_string(unsigned(param.get_type())).c_str(), param.get_name().c_str());
    }
  } else if (param.get_name() == "output_encoding") {
    output_encoding_ = param.as_string();
    return true;
  } else if (param.get_name() == "pixel_format") {
    camera_->stop();
    auto success = requestPixelFormat(param.as_string());
    camera_->start();
    return success;
  } else if (param.get_name() == "image_size") {
    camera_->stop();
    auto success = requestImageSize(param.as_integer_array());
    camera_->start();
    return success;
  } else if (param.get_name() == "time_per_frame") {
    camera_->stop();
    auto success = requestTimePerFrame(param.as_integer_array());
    camera_->start();
    return success;
  } else if (param.get_name() == "camera_info_url") {
    auto camera_info_url = param.as_string();
    if (cinfo_->validateURL(camera_info_url)) {
      return cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      return false;
    }
  }

  return false;
}

bool V4L2Camera::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Invalid pixel format size: must be a 4 character code (FOURCC).");
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
    RCLCPP_WARN(
      get_logger(),
      "Invalid image size; expected dimensions: 2, actual: %d",
      size.size());
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
    RCLCPP_WARN(
      get_logger(),
      "Invalid time per frame; expected dimensions: 2, actual: %d",
      tpf.size());
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

sensor_msgs::msg::Image::UniquePtr V4L2Camera::convert(sensor_msgs::msg::Image const & img) const
{
  RCLCPP_DEBUG(
    get_logger(),
    "Converting: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());

  // TODO(sander): temporary until cv_bridge and image_proc are available in ROS 2
  if (img.encoding == sensor_msgs::image_encodings::YUV422 &&
    output_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    auto outImg = std::make_unique<sensor_msgs::msg::Image>();
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
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Conversion not supported yet: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());
    return nullptr;
  }
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

}  // namespace v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
