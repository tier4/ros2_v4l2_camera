#include "gpu_imgproc/gpu_imgproc.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <future>

namespace gpu_imgproc {

GpuImgProc::GpuImgProc(const rclcpp::NodeOptions & options)
    : Node("gpu_imgproc", options), rectifier_active_(false) {
    RCLCPP_INFO(this->get_logger(), "Initializing node gpu_imgproc");

    // std::string image_raw_topic = this->declare_parameter<std::string>("image_raw_topic", "/camera/image_raw");
    // std::string camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    // std::string image_rect_topic = this->declare_parameter<std::string>("image_rect_topic", "/camera/image_rect");
    std::string rect_impl = this->declare_parameter<std::string>("rect_impl", "npp");
    bool use_opencv_map_init = this->declare_parameter<bool>("use_opencv_map_init", false);
    alpha_ = this->declare_parameter<double>("alpha", 0.0);

    // RCLCPP_INFO(this->get_logger(), "Subscribing to %s", image_raw_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "Subscribing to %s", camera_info_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "Publishing to %s", image_rect_topic.c_str());

    std::string available_impls = "";
#ifdef NPP_AVAILABLE
    available_impls += "npp";
#endif
#ifdef OPENCV_AVAILABLE
    if (available_impls != "") {
        available_impls += ", ";
    }
    available_impls += "opencv_cpu";
#endif
#ifdef OPENCV_CUDA_AVAILABLE
    if (available_impls != "") {
        available_impls += ", ";
    }
    available_impls += "opencv_gpu";
#endif

    if (available_impls == "") {
        RCLCPP_ERROR(this->get_logger(),
        "No rectification implementations available. Please make sure that at least one of the following libraries is installed:\n"
        "- OpenCV\n"
        "- OpenCV CUDA\n"
        "- NVIDIA Performance Primitives\n");
        return;
    }

    if (0) {
#ifdef NPP_AVAILABLE
    } else if (rect_impl == "npp") {
        RCLCPP_INFO(this->get_logger(), "Using NPP implementation for rectification");
        rectifier_impl_ = Rectifier::Implementation::NPP;
#endif
#ifdef OPENCV_AVAILABLE
    } else if (rect_impl == "opencv_cpu") {
        RCLCPP_INFO(this->get_logger(), "Using CPU OpenCV implementation for rectification");
        rectifier_impl_ = Rectifier::Implementation::OpenCV_CPU;
#endif
#ifdef OPENCV_CUDA_AVAILABLE
    } else if (rect_impl == "opencv_gpu") {
        RCLCPP_INFO(this->get_logger(), "Using GPU OpenCV implementation for rectification");
        rectifier_impl_ = Rectifier::Implementation::OpenCV_GPU;
#endif
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid implementation: %s. Available options: %s", rect_impl.c_str(), available_impls.c_str());
        return;
    }

    if (use_opencv_map_init) {
        RCLCPP_INFO(this->get_logger(), "Using OpenCV map initialization");
        mapping_impl_ = Rectifier::MappingImpl::OpenCV;
    } else {
        RCLCPP_INFO(this->get_logger(), "Using Non-OpenCV map initialization");
        mapping_impl_ = Rectifier::MappingImpl::NPP;
    }

#ifdef JETSON_AVAILABLE
    raw_compressor_ = std::make_shared<JpegCompressor::JetsonCompressor>("raw_compressor");
    rect_compressor_ = std::make_shared<JpegCompressor::JetsonCompressor>("rect_compressor");
#elif NVJPEG_AVAILABLE
    raw_compressor_ = std::make_shared<JpegCompressor::NVJPEGCompressor>();
    rect_compressor_ = std::make_shared<JpegCompressor::NVJPEGCompressor>();
#elif TURBOJPEG_AVAILABLE
    raw_compressor_ = std::make_shared<JpegCompressor::CPUCompressor>();
    rect_compressor_ = std::make_shared<JpegCompressor::CPUCompressor>();
#else
    RCLCPP_ERROR(this->get_logger(), "No JPEG compressor available");
    return;
#endif

    rectified_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "image_rect", 10);
    compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", 10);
    rect_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_rect/compressed", 10);

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&GpuImgProc::imageCallback, this, std::placeholders::_1));
    
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&GpuImgProc::cameraInfoCallback, this, std::placeholders::_1));
}

GpuImgProc::~GpuImgProc() {
    RCLCPP_INFO(this->get_logger(), "Shutting down node gpu_imgproc");
}

void GpuImgProc::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received image");

    std::future<void> rectified_msg;
    if (rectifier_active_) {
        RCLCPP_DEBUG(this->get_logger(), "Rectifying image");
        rectified_msg =
            std::async(std::launch::async, [this, msg]() {
                sensor_msgs::msg::Image::UniquePtr rect_img;
                sensor_msgs::msg::CompressedImage::UniquePtr rect_comp_img;
                if (false) {
#ifdef NPP_AVAILABLE
                } else if (rectifier_impl_ == Rectifier::Implementation::NPP) {
                    rect_img = npp_rectifier_->rectify(*msg);
                    rect_comp_img = rect_compressor_->compress(*rect_img, 60);
#endif
#ifdef OPENCV_AVAILABLE                            
                } else if (rectifier_impl_ == Rectifier::Implementation::OpenCV_CPU) {
                    rect_img = cv_cpu_rectifier_->rectify(*msg);
                    rect_comp_img = rect_compressor_->compress(*rect_img, 60);
#endif
#ifdef OPENCV_CUDA_AVAILABLE
                } else if (rectifier_impl_ == Rectifier::Implementation::OpenCV_GPU) {
                    rect_img = cv_gpu_rectifier_->rectify(*msg);
                    rect_comp_img = rect_compressor_->compress(*rect_img, 60);
#endif
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Invalid implementation");
                    return;
                }
                rectified_pub_->publish(std::move(rect_img));
                rect_compressed_pub_->publish(std::move(rect_comp_img));
            });
    } else {
        std::cout << "Not rectifying image" << std::endl;
    }

    std::future<void> compressed_msg =
        std::async(std::launch::async, [this, msg]() {
            compressed_pub_->publish(std::move(raw_compressor_->compress(*msg, 60)));
        });
    
    if (rectifier_active_) {
        rectified_msg.wait();
    }
    compressed_msg.wait();
}

void GpuImgProc::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received camera info");

    if (msg->d.size() == 0 || msg->p.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Camera info message does not contain distortion or projection matrix");
        return;
    }

    switch(rectifier_impl_) {
        case Rectifier::Implementation::NPP:
#if NPP_AVAILABLE
            RCLCPP_INFO(this->get_logger(), "Initializing NPP rectifier");
            npp_rectifier_ = std::make_shared<Rectifier::NPPRectifier>(*msg, mapping_impl_, alpha_);
            if (npp_rectifier_) {
                RCLCPP_INFO(this->get_logger(), "Initialized NPP rectifier");
                rectifier_active_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize NPP rectifier");
                return;
            }
            break;
#else
            RCLCPP_ERROR(this->get_logger(), "NPP not enabled");
            return;
#endif
        case Rectifier::Implementation::OpenCV_CPU:
#ifdef OPENCV_AVAILABLE
            RCLCPP_INFO(this->get_logger(), "Initializing OpenCV CPU rectifier");
            cv_cpu_rectifier_ = std::make_shared<Rectifier::OpenCVRectifierCPU>(*msg, mapping_impl_, alpha_);
            if (cv_cpu_rectifier_) {
                RCLCPP_INFO(this->get_logger(), "Initialized OpenCV GPU rectifier");
                rectifier_active_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenCV rectifier");
                return;
            }
            break;
#else
            RCLCPP_ERROR(this->get_logger(), "OpenCV not enabled");
            return;
#endif 
        case Rectifier::Implementation::OpenCV_GPU:
#ifdef OPENCV_CUDA_AVAILABLE
            RCLCPP_INFO(this->get_logger(), "Initializing OpenCV GPU rectifier");
            cv_gpu_rectifier_ = std::make_shared<Rectifier::OpenCVRectifierGPU>(*msg, mapping_impl_, alpha_);
            if (cv_gpu_rectifier_) {
                RCLCPP_INFO(this->get_logger(), "Initialized OpenCV GPU rectifier");
                rectifier_active_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenCV rectifier");
                return;
            }
            break;
#else
            RCLCPP_ERROR(this->get_logger(), "OpenCV CUDA not enabled");
            return;
#endif
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid rectifier implementation");
            return;
    }

    if (rectifier_active_) {
        // unsubscribe
        info_sub_.reset();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(GpuImgProc)
} // namespace gpu_imgproc