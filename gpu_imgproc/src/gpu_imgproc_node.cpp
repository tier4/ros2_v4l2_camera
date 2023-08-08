#include <ros/ros.h>
#include "gpu_imgproc/gpu_imgproc.hpp"


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gpu_imgproc");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  gpu_imgproc::GpuImgProc n(nh, nh_priv);

  ros::spin();
  
  return 0;
}