#pragma once

#include <rclcpp/rclcpp.hpp>
// #include <rcl_interfaces/msg/parameter.hpp>

#include "accelerator/rectifier.hpp"
#include "accelerator/jpeg_compressor.hpp"
// #include <sensor_msgs/msg/compressed_image.hpp>

namespace gpu_imgproc {

class GpuImgProc : public rclcpp::Node {
public:
    explicit GpuImgProc(const rclcpp::NodeOptions & options);
    virtual ~GpuImgProc();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rectified_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rect_compressed_pub_;

    Rectifier::Implementation rectifier_impl_;
    Rectifier::MappingImpl mapping_impl_;
    bool rectifier_active_;
    double alpha_;
};


} // namespace gpu_imgproc