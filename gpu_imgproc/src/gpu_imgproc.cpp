#include "gpu_imgproc/gpu_imgproc.hpp"

#include <future>

namespace gpu_imgproc {

GpuImgProc::GpuImgProc(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : node_(nh), private_node_(pnh), rectifier_active_(false) {
    ROS_INFO("Initializing node gpu_imgproc");

    // std::string image_raw_topic = this->declare_parameter<std::string>("image_raw_topic", "/camera/image_raw");
    // std::string camera_info_topic = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    // std::string image_rect_topic = this->declare_parameter<std::string>("image_rect_topic", "/camera/image_rect");
    std::string rect_impl;
    bool use_opencv_map_init;

    private_node_.getParam("rect_impl", rect_impl);
    private_node_.getParam("use_opencv_map_init", use_opencv_map_init);
    private_node_.getParam("alpha", alpha_);

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
        ROS_ERROR(
        "No rectification implementations available. Please make sure that at least one of the following libraries is installed:\n"
        "- OpenCV\n"
        "- OpenCV CUDA\n"
        "- NVIDIA Performance Primitives\n");
        return;
    }

    if (0) {
#ifdef NPP_AVAILABLE
    } else if (rect_impl == "npp") {
        ROS_INFO("Using NPP implementation for rectification");
        rectifier_impl_ = Rectifier::Implementation::NPP;
#endif
#ifdef OPENCV_AVAILABLE
    } else if (rect_impl == "opencv_cpu") {
        ROS_INFO("Using OpenCV CPU implementation for rectification");
        rectifier_impl_ = Rectifier::Implementation::OpenCV_CPU;
#endif
#ifdef OPENCV_CUDA_AVAILABLE
    } else if (rect_impl == "opencv_gpu") {
        ROS_INFO("Using GPU OpenCV implementation for rectification");
        rectifier_impl_ = Rectifier::Implementation::OpenCV_GPU;
#endif
    } else {
        ROS_INFO("Available implementations: %s", available_impls.c_str());
        return;
    }

    if (use_opencv_map_init) {
        ROS_INFO("Using OpenCV map initialization");
        mapping_impl_ = Rectifier::MappingImpl::OpenCV;
    } else {
        ROS_INFO("Using Non-OpenCV map initialization");
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
    ROS_ERROR("No JPEG compressor available");
    return;
#endif

    rectified_pub_ = node_.advertise<Image>("image_rect", 10);
    compressed_pub_ = node_.advertise<CompressedImage>("image_raw/compressed", 10);
    rect_compressed_pub_ = node_.advertise<CompressedImage>("image_rect/compressed", 10);

    img_sub_ = node_.subscribe("image_raw", 10, &GpuImgProc::imageCallback, this);
    
    info_sub_ = node_.subscribe(
        "camera_info", 10, &GpuImgProc::cameraInfoCallback, this
    );
}

GpuImgProc::~GpuImgProc() {
    ROS_INFO("Shutting down node gpu_imgproc");
}

void GpuImgProc::imageCallback(const Image::ConstPtr &msg) {
    // RCLCPP_INFO(this->get_logger(), "Received image");

    std::future<void> rectified_msg;
    if (rectifier_active_) {
        // std::cout << "Rectifying image" << std::endl;
        rectified_msg =
            std::async(std::launch::async, [this, msg]() {
                ImagePtr rect_img;
                CompressedImagePtr rect_comp_img;
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
                    ROS_ERROR("Invalid implementation");
                    return;
                }
                
                // TODO: Do not copy on publish
                rectified_pub_.publish(*rect_img);
                rect_compressed_pub_.publish(*rect_comp_img);
            });
    } else {
        std::cout << "Not rectifying image" << std::endl;
    }

    std::future<void> compressed_msg =
        std::async(std::launch::async, [this, msg]() {
            compressed_pub_.publish(*raw_compressor_->compress(*msg, 60));
        });
    
    if (rectifier_active_) {
        rectified_msg.wait();
    }
    compressed_msg.wait();
}

void GpuImgProc::cameraInfoCallback(const CameraInfo::ConstPtr &msg) {
    ROS_INFO("Received camera info");

    if (msg->D.size() == 0 || msg->P.size() == 0) {
        ROS_ERROR("Camera info message does not contain distortion or projection matrix");
        return;
    }

    switch(rectifier_impl_) {
        case Rectifier::Implementation::NPP:
#if NPP_AVAILABLE
            ROS_INFO("Initializing NPP rectifier");
            npp_rectifier_ = std::make_shared<Rectifier::NPPRectifier>(*msg, mapping_impl_, alpha_);
            if (npp_rectifier_) {
                ROS_INFO("Initialized NPP rectifier");
                rectifier_active_ = true;
            } else {
                ROS_ERROR("Failed to initialize NPP rectifier");
                return;
            }
            break;
#else
            ROS_ERROR("NPP not enabled");
            return;
#endif
        case Rectifier::Implementation::OpenCV_CPU:
#ifdef OPENCV_AVAILABLE
            ROS_INFO("Initializing OpenCV CPU rectifier");
            cv_cpu_rectifier_ = std::make_shared<Rectifier::OpenCVRectifierCPU>(*msg, mapping_impl_, alpha_);
            if (cv_cpu_rectifier_) {
                ROS_INFO("Initialized OpenCV CPU rectifier");
                rectifier_active_ = true;
            } else {
                ROS_ERROR("Failed to initialize OpenCV rectifier");
                return;
            }
            break;
#else
            ROS_ERROR("OpenCV not enabled");
            return;
#endif 
        case Rectifier::Implementation::OpenCV_GPU:
#ifdef OPENCV_CUDA_AVAILABLE
            ROS_INFO("Initializing OpenCV GPU rectifier");
            cv_gpu_rectifier_ = std::make_shared<Rectifier::OpenCVRectifierGPU>(*msg, mapping_impl_, alpha_);
            if (cv_gpu_rectifier_) {
                ROS_INFO("Initialized OpenCV GPU rectifier");
                rectifier_active_ = true;
            } else {
                ROS_ERROR("Failed to initialize OpenCV rectifier");
                return;
            }
            break;
#else
            ROS_ERROR("OpenCV CUDA not enabled");
            return;
#endif
        default:
            ROS_ERROR("Invalid rectifier implementation");
            return;
    }

    if (rectifier_active_) {
        // unsubscribe
        // info_sub_.reset();
        // TODO: ROS1 equivalent?
        info_sub_.shutdown();
    }
}
} // namespace gpu_imgproc