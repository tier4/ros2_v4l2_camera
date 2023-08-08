#pragma once

#include <ros/ros.h>
// #include <rcl_interfaces/msg/parameter.hpp>

#include "accelerator/rectifier.hpp"
#include "accelerator/jpeg_compressor.hpp"
// #include <sensor_msgs/msg/compressed_image.hpp>

namespace gpu_imgproc {

using Image = sensor_msgs::Image;
using CompressedImage = sensor_msgs::CompressedImage;
using CompressedImagePtr = std::shared_ptr<CompressedImage>;
using CameraInfo = sensor_msgs::CameraInfo;

class GpuImgProc {
public:
    explicit GpuImgProc(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    virtual ~GpuImgProc();

private:
    void imageCallback(const Image::ConstPtr &msg);
    void cameraInfoCallback(const CameraInfo::ConstPtr &msg);

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;

#if NPP_AVAILABLE
    std::shared_ptr<Rectifier::NPPRectifier> npp_rectifier_;
#endif
#ifdef OPENCV_AVAILABLE
    std::shared_ptr<Rectifier::OpenCVRectifierCPU> cv_cpu_rectifier_;
#endif
#ifdef OPENCV_CUDA_AVAILABLE
    std::shared_ptr<Rectifier::OpenCVRectifierGPU> cv_gpu_rectifier_;
#endif
#ifdef JETSON_AVAILABLE
    std::shared_ptr<JpegCompressor::JetsonCompressor> raw_compressor_;
    std::shared_ptr<JpegCompressor::JetsonCompressor> rect_compressor_;
#elif NVJPEG_AVAILABLE
    std::shared_ptr<JpegCompressor::NVJPEGCompressor> raw_compressor_;
    std::shared_ptr<JpegCompressor::NVJPEGCompressor> rect_compressor_;
#elif TURBOJPEG_AVAILABLE
    std::shared_ptr<JpegCompressor::CPUCompressor> raw_compressor_;
    std::shared_ptr<JpegCompressor::CPUCompressor> rect_compressor_;
#endif

    ros::Subscriber img_sub_;
    ros::Subscriber info_sub_;

    ros::Publisher rectified_pub_;
    ros::Publisher compressed_pub_;
    ros::Publisher rect_compressed_pub_;

    Rectifier::Implementation rectifier_impl_;
    Rectifier::MappingImpl mapping_impl_;
    bool rectifier_active_;
    double alpha_;
};


} // namespace gpu_imgproc