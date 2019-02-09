#include "ros2_v4l2_camera/v4l2_camera.hpp"

using namespace ros2_v4l2_camera;
using namespace sensor_msgs::msg;

V4l2Camera::V4l2Camera(std::string device)
{
}

void V4l2Camera::open()
{
}

void V4l2Camera::start()
{
}

void V4l2Camera::stop()
{
}

Image V4l2Camera::capture()
{
  auto img = Image{};
  img.width = 640;
  img.height = 480;
  img.encoding = "mono8";
  img.data.resize(640 * 480);
  std::fill(img.data.begin(), img.data.end(), 128);
  return img;
}
