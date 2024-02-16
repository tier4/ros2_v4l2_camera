#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <memory>

#include "v4l2_camera/v4l2_camera.hpp"
#include "v4l2_camera/v4l2_camera_device.hpp"

namespace v4l2_camera
{
class V4L2CameraNodelet : public nodelet::Nodelet
{
	public:
		V4L2CameraNodelet(){}
		~V4L2CameraNodelet(){}

	private:
		virtual void onInit(void);
  	std::shared_ptr<v4l2_camera::V4L2Camera> camera_;
};

void V4L2CameraNodelet::onInit()
{
  camera_.reset(new v4l2_camera::V4L2Camera(getNodeHandle(), getPrivateNodeHandle()));
}

}  // namespace v4l2_camera

PLUGINLIB_EXPORT_CLASS(v4l2_camera::V4L2CameraNodelet, nodelet::Nodelet)
