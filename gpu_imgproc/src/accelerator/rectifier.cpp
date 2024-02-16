#include <ros/ros.h>
#include "accelerator/rectifier.hpp"
// #include <rclcpp/rclcpp.hpp>

#if NPP_AVAILABLE
#include <npp.h>
#include <nppi_data_exchange_and_initialization.h>
#include <nppi_geometry_transforms.h>
#include <nppi_support_functions.h>
#endif

#ifdef OPENCV_AVAILABLE
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#ifdef OPENCV_CUDA_AVAILABLE
// #include <opencv2/cudafeatures2d.hpp>
// #include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/core.hpp>
#endif
#endif


#define CHECK_NPP(status) \
    if (status != NPP_SUCCESS) { \
        ROS_ERROR("NPP failure: '%d' at %s:%d", status, __FILE__, __LINE__); \
    }

#define CHECK_CUDA(status) \
    if (status != cudaSuccess) { \
        ROS_ERROR("CUDA failure: '%s' at %s:%d", cudaGetErrorName(status), __FILE__, __LINE__); \
    }

namespace Rectifier {

static void compute_maps(int width, int height, const double *D, const double *P,
                  float *map_x, float *map_y) {
    ROS_WARN("No support for alpha in non-OpenCV mapping");

    double fx = P[0];
    double fy = P[5];
    double cx = P[2];
    double cy = P[6];

    double k1 = D[0];
    double k2 = D[1];
    double p1 = D[2];
    double p2 = D[3];
    double k3 = D[4];

    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double r2 = x * x + y * y;
            double r4 = r2 * r2;
            double r6 = r4 * r2;
            double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
            double xd = x * cdist;
            double yd = y * cdist;
            double x2 = xd * xd;
            double y2 = yd * yd;
            double xy = xd * yd;
            double kr = 1 + p1 * r2 + p2 * r4;
            map_x[v * width + u] = (float)(fx * (xd * kr + 2 * p1 * xy + p2 * (r2 + 2 * x2)) + cx);
            map_y[v * width + u] = (float)(fy * (yd * kr + p1 * (r2 + 2 * y2) + 2 * p2 * xy) + cy);
        }
    }
}

#ifdef OPENCV_AVAILABLE
static void compute_maps_opencv(const CameraInfo &info, float *map_x, float *map_y, double alpha = 0.0) {
    cv::Mat camera_intrinsics(3, 3, CV_64F);
    cv::Mat distortion_coefficients(1, 5, CV_64F);

    for (int row=0; row<3; row++) {
        for (int col=0; col<3; col++) {
            camera_intrinsics.at<double>(row, col) = info.K[row * 3 + col];
        }
    }

    for (int col=0; col<5; col++) {
        distortion_coefficients.at<double>(col) = info.D[col];
    }

    cv::Mat new_intrinsics = cv::getOptimalNewCameraMatrix(camera_intrinsics,
        distortion_coefficients,
        cv::Size(info.width, info.height),
        alpha);

    cv::Mat m1(info.height, info.width, CV_32FC1, map_x);
    cv::Mat m2(info.height, info.width, CV_32FC1, map_y);
    
    cv::initUndistortRectifyMap(camera_intrinsics,
        distortion_coefficients,
        cv::Mat(),
        new_intrinsics,
        cv::Size(info.width, info.height),
        CV_32FC1,
        m1, m2);
}
#endif

#if NPP_AVAILABLE
NPPRectifier::NPPRectifier(int width, int height,
                           const Npp32f *map_x, const Npp32f *map_y)
    : pxl_map_x_(nullptr), pxl_map_y_(nullptr) {
    cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);

    pxl_map_x_ = nppiMalloc_32f_C1(width, height, &pxl_map_x_step_);
    if (pxl_map_x_ == nullptr) {
        ROS_ERROR("Failed to allocate GPU memory");
        return;
    }
    pxl_map_y_ = nppiMalloc_32f_C1(width, height, &pxl_map_y_step_);
    if (pxl_map_y_ == nullptr) {
        ROS_ERROR("Failed to allocate GPU memory");
        return;
    }
    CHECK_CUDA(cudaMemcpy2D(pxl_map_x_, pxl_map_x_step_, map_x, width * sizeof(float), width * sizeof(float), height, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy2D(pxl_map_y_, pxl_map_y_step_, map_y, width * sizeof(float), width * sizeof(float), height, cudaMemcpyHostToDevice));
}

NPPRectifier::NPPRectifier(const CameraInfo& info, MappingImpl impl, double alpha) {
    cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);

    nppSetStream(stream_);

    pxl_map_x_ = nppiMalloc_32f_C1(info.width, info.height, &pxl_map_x_step_);
    if (pxl_map_x_ == nullptr) {
        ROS_ERROR("Failed to allocate GPU memory");
        return;
    }
    pxl_map_y_ = nppiMalloc_32f_C1(info.width, info.height, &pxl_map_y_step_);
    if (pxl_map_y_ == nullptr) {
        ROS_ERROR("Failed to allocate GPU memory");
        return;
    }

    std::cout << "Rectifying image with " << info.width << "x" << info.height << " pixels" << std::endl;

    // Create rectification map
    // TODO: Verify this works
    float *map_x = new float[info.width * info.height];
    float *map_y = new float[info.width * info.height];

#ifdef OPENCV_AVAILABLE
    if (impl == MappingImpl::OpenCV)
        compute_maps_opencv(info, map_x, map_y, alpha);
    else
#endif
    compute_maps(info.width, info.height,
                 info.D.data(), info.P.data(),
                 map_x, map_y);

    std::cout << "Copying rectification map to GPU" << std::endl;

    CHECK_CUDA(cudaMemcpy2D(pxl_map_x_, pxl_map_x_step_, map_x, info.width * sizeof(float), info.width * sizeof(float), info.height, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy2D(pxl_map_y_, pxl_map_y_step_, map_y, info.width * sizeof(float), info.width * sizeof(float), info.height, cudaMemcpyHostToDevice));

    delete[] map_x;
    delete[] map_y;
}

NPPRectifier::~NPPRectifier() {
    if (pxl_map_x_ != nullptr) {
        nppiFree(pxl_map_x_);
    }

    if (pxl_map_y_ != nullptr) {
        nppiFree(pxl_map_y_);
    }

    cudaStreamDestroy(stream_);
}

ImagePtr NPPRectifier::rectify(const Image &msg) {
    ImagePtr result = std::make_shared<Image>();
    result->header = msg.header;
    result->height = msg.height;
    result->width = msg.width;
    result->encoding = msg.encoding;
    result->is_bigendian = msg.is_bigendian;
    result->step = msg.step;

    result->data.resize(msg.data.size());

    NppiRect src_roi = {0, 0, (int)msg.width, (int)msg.height};
    NppiSize src_size = {(int)msg.width, (int)msg.height};
    NppiSize dst_roi_size = {(int)msg.width, (int)msg.height};
    int src_step;
    int dst_step;

    Npp8u *src = nppiMalloc_8u_C3(msg.width, msg.height, &src_step);
    Npp8u *dst = nppiMalloc_8u_C3(msg.width, msg.height, &dst_step);

    CHECK_CUDA(cudaMemcpy2D(src, src_step, msg.data.data(), msg.step, msg.width * 3, msg.height, cudaMemcpyHostToDevice));

    NppiInterpolationMode interpolation = NPPI_INTER_LINEAR;

    CHECK_NPP(nppiRemap_8u_C3R(
        src, src_size, src_step, src_roi,
        pxl_map_x_, pxl_map_x_step_, pxl_map_y_, pxl_map_y_step_,
        dst, dst_step, dst_roi_size, interpolation));

    CHECK_CUDA(cudaMemcpy2D(result->data.data(), result->step, dst, dst_step, msg.width * 3, msg.height, cudaMemcpyDeviceToHost));

    // cv::Mat image(msg.height, msg.width, CV_8UC3, result->data.data(), result->step);
    // cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    // // save with timestamp
    // std::string filename = "rectified_" + std::to_string(msg.header.stamp.sec) + "_" + std::to_string(msg.header.stamp.nanosec) + ".png";
    // imwrite(filename, image);

    nppiFree(src);
    nppiFree(dst);

    return result;
}
#endif

#ifdef OPENCV_AVAILABLE
OpenCVRectifierCPU::OpenCVRectifierCPU(const CameraInfo &info, MappingImpl impl, double alpha) {
    map_x_ = cv::Mat(info.height, info.width, CV_32FC1);
    map_y_ = cv::Mat(info.height, info.width, CV_32FC1);

    if (impl == MappingImpl::OpenCV)
        compute_maps_opencv(info, map_x_.ptr<float>(), map_y_.ptr<float>(), alpha);
    else
        compute_maps(info.width, info.height,
                     info.D.data(), info.P.data(),
                     map_x_.ptr<float>(), map_y_.ptr<float>());
}

OpenCVRectifierCPU::~OpenCVRectifierCPU() {}

ImagePtr OpenCVRectifierCPU::rectify(const Image &msg) {
    ImagePtr result = std::make_shared<Image>();
    result->header = msg.header;
    result->height = msg.height;
    result->width = msg.width;
    result->encoding = msg.encoding;
    result->is_bigendian = msg.is_bigendian;
    result->step = msg.step;

    result->data.resize(msg.data.size());

    cv::Mat src(msg.height, msg.width, CV_8UC3, (void *)msg.data.data());
    cv::Mat dst(msg.height, msg.width, CV_8UC3, (void *)result->data.data());

    cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR);

    return result;
}
#endif

#ifdef OPENCV_CUDA_AVAILABLE
OpenCVRectifierGPU::OpenCVRectifierGPU(const CameraInfo &info, MappingImpl impl, double alpha) {
    cv::Mat map_x(info.height, info.width, CV_32FC1);
    cv::Mat map_y(info.height, info.width, CV_32FC1);

    if (impl == MappingImpl::OpenCV)
        compute_maps_opencv(info, map_x.ptr<float>(), map_y.ptr<float>(), alpha);
    else
        compute_maps(info.width, info.height,
                     info.D.data(), info.P.data(),
                     map_x.ptr<float>(), map_y.ptr<float>());

    map_x_ = cv::cuda::GpuMat(map_x);
    map_y_ = cv::cuda::GpuMat(map_y);
}

OpenCVRectifierGPU::~OpenCVRectifierGPU() {}

ImagePtr OpenCVRectifierGPU::rectify(const Image &msg) {
    ImagePtr result = std::make_shared<Image>();
    result->header = msg.header;
    result->height = msg.height;
    result->width = msg.width;
    result->encoding = msg.encoding;
    result->is_bigendian = msg.is_bigendian;
    result->step = msg.step;

    result->data.resize(msg.data.size());

    cv::Mat src(msg.height, msg.width, CV_8UC3, (void *)msg.data.data());
    cv::cuda::GpuMat d_src = cv::cuda::GpuMat(src);
    cv::cuda::GpuMat d_dst = cv::cuda::GpuMat(cv::Size(msg.width, msg.height), src.type());

    cv::cuda::remap(d_src, d_dst, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // copy back to result
    cv::Mat dst(msg.height, msg.width, CV_8UC3, (void *)result->data.data());
    d_dst.download(dst);

    // cv::Mat image(msg.height, msg.width, CV_8UC3, result->data.data(), result->step);
    // cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    // // save with timestamp
    // std::string filename = "rectified_" + std::to_string(msg.header.stamp.sec) + "_" + std::to_string(msg.header.stamp.nanosec) + ".png";
    // imwrite(filename, image);

    return result;
}
#endif

} // namespace Rectifier