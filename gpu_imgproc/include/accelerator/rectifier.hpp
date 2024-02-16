#pragma once

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/msg/camera_info.hpp>

#ifdef OPENCV_AVAILABLE
#include <opencv2/core.hpp>
#endif
#ifdef OPENCV_CUDA_AVAILABLE
#include <opencv2/core/cuda.hpp>
#endif

#if NPP_AVAILABLE
#include <nppdefs.h>
#endif

using CameraInfo = sensor_msgs::CameraInfo;
using Image = sensor_msgs::Image;
typedef std::shared_ptr<Image> ImagePtr;

namespace Rectifier {

enum class Implementation {
    NPP,
    OpenCV_CPU,
    OpenCV_GPU
};

enum class MappingImpl {
    NPP,
    OpenCV
};

#if NPP_AVAILABLE
class NPPRectifier {
public:
    NPPRectifier(int width, int height,
                 const Npp32f *map_x, const Npp32f *map_y);
    NPPRectifier(const CameraInfo &info,
                 MappingImpl impl = MappingImpl::NPP,
                 double alpha = 0.0);
    ~NPPRectifier();

    ImagePtr rectify(const Image &msg);
private:
    Npp32f *pxl_map_x_;
    Npp32f *pxl_map_y_;
    int pxl_map_x_step_;
    int pxl_map_y_step_;
    int interpolation_;
    cudaStream_t stream_;
};
#endif

#ifdef OPENCV_AVAILABLE
class OpenCVRectifierCPU {
public:
    OpenCVRectifierCPU(const CameraInfo &info,
                       MappingImpl impl = MappingImpl::OpenCV,
                       double alpha = 0.0);
    ~OpenCVRectifierCPU();

    ImagePtr rectify(const Image &msg);
private:
    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Mat camera_intrinsics_;
    cv::Mat distortion_coeffs_;
};
#endif

#ifdef OPENCV_CUDA_AVAILABLE
class OpenCVRectifierGPU {
public:
    OpenCVRectifierGPU(const CameraInfo &info,
                       MappingImpl impl = MappingImpl::OpenCV,
                       double alpha = 0.0);
    ~OpenCVRectifierGPU();

    ImagePtr rectify(const Image &msg);
private:
    cv::cuda::GpuMat map_x_;
    cv::cuda::GpuMat map_y_;
};
#endif

} // namespace Rectifier