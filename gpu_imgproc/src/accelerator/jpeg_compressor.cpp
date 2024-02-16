#include <cstdio>
#include <cstring>

#include <ros/ros.h>

#include "accelerator/jpeg_compressor.hpp"
#include "accelerator/color_space.hpp"

#if defined(JETSON_AVAILABLE) || defined(NVJPEG_AVAILABLE)
#include <nppi_color_conversion.h>
#endif

#define TEST_ERROR(cond, str) if(cond) { \
                                        fprintf(stderr, "%s\n", str); }

#define CHECK_CUDA(status) \
    if (status != cudaSuccess) { \
        ROS_ERROR("CUDA failure: '%s' at %s:%d", cudaGetErrorString(status), __FILE__, __LINE__); \
    }

#define CHECK_NVJPEG(call)                                                                      \
    {                                                                                           \
        nvjpegStatus_t _e = (call);                                                             \
        if (_e != NVJPEG_STATUS_SUCCESS) {                                                      \
            ROS_ERROR("NVJPEG failure: '%d' at %s:%d", _e, __FILE__, __LINE__); \
            exit(1);                                                                            \
        }                                                                                       \
    }

namespace JpegCompressor {

#ifdef TURBOJPEG_AVAILABLE
CPUCompressor::CPUCompressor()
    : jpegBuf_(nullptr), size_(0) {
    handle_ = tjInitCompress();
}

CPUCompressor::~CPUCompressor() {
    if (jpegBuf_)
        tjFree(jpegBuf_);
    tjDestroy(handle_);
}

CompressedImagePtr CPUCompressor::compress(const Image &msg, int quality, int format, int sampling) {
    CompressedImagePtr compressed_msg = std::make_shared<CompressedImage>();
    compressed_msg->header = msg.header;
    compressed_msg->format = "jpeg";

    if (jpegBuf_) {
        tjFree(jpegBuf_);
        jpegBuf_ = nullptr;
    }

    int tjres = tjCompress2(handle_,
                            msg.data.data(),
                            msg.width,
                            0,
                            msg.height,
                            format,
                            &jpegBuf_,
                            &size_,
                            sampling,
                            quality,
                            TJFLAG_FASTDCT);

    TEST_ERROR(tjres != 0, tjGetErrorStr2(handle_));

    compressed_msg->data.resize(size_);
    memcpy(compressed_msg->data.data(), jpegBuf_, size_);

    return compressed_msg;
}
#endif

#ifdef JETSON_AVAILABLE
JetsonCompressor::JetsonCompressor(std::string name) {
    encoder_ = NvJPEGEncoder::createJPEGEncoder(name.c_str());
}

JetsonCompressor::~JetsonCompressor() {
    delete encoder_;
}

CompressedImagePtr JetsonCompressor::compress(const Image &msg, int quality, ImageFormat format) {
    CompressedImagePtr compressed_msg = std::make_shared<CompressedImage>();
    compressed_msg->header = msg.header;
    compressed_msg->format = "jpeg";

    int width = msg.width;
    int height = msg.height;
    const auto &img = msg.data;

    if (image_size < img.size()) {
      dev_image = cuda::memory::device::make_unique<uint8_t[]>(img.size());
      yuv_size =
          width * height +
          (static_cast<size_t>(width / 2) * static_cast<size_t>(height / 2)) * 2;
      host_yuv = cuda::memory::host::make_unique<uint8_t[]>(yuv_size);
      dev_yuv = cuda::memory::device::make_unique<uint8_t[]>(yuv_size);
      image_size = img.size();
    }

    cuda::memory::copy(dev_image.get(), img.data(), img.size());

    if (format == ImageFormat::RGB) {
        TEST_ERROR(cudaRGBToYUV420(dev_image.get(), dev_yuv.get(), width, height) !=
                cudaSuccess, "failed to convert rgb8 to yuv420");
    } else {
        TEST_ERROR(cudaBGRToYUV420(dev_image.get(), dev_yuv.get(), width, height) !=
                cudaSuccess, "failed to convert bgr8 to yuv420");
    }

    cuda::memory::copy(host_yuv.get(), dev_yuv.get(), yuv_size);

    NvBuffer buffer(V4L2_PIX_FMT_YUV420M, width, height, 0);
    TEST_ERROR(buffer.allocateMemory() != 0, "NvBuffer allocation failed");

    auto image_data = reinterpret_cast<int8_t *>(host_yuv.get());

    for (uint32_t i = 0; i < buffer.n_planes; ++i) {
        NvBuffer::NvBufferPlane &plane = buffer.planes[i];
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
        memcpy(plane.data, image_data, plane.bytesused);
        image_data += plane.bytesused;
    }

    size_t out_buf_size = width * height * 3 / 2;
    compressed_msg->data.resize(out_buf_size);
    auto out_data = compressed_msg->data.data();

    TEST_ERROR(
        encoder_->encodeFromBuffer(buffer, JCS_YCbCr, &out_data,
                                   out_buf_size, quality),
        "NvJpeg Encoder Error");

    buffer.deallocateMemory();

    compressed_msg->data.resize(out_buf_size);
    
    return compressed_msg;
}
#endif

#ifdef NVJPEG_AVAILABLE
NVJPEGCompressor::NVJPEGCompressor() {
    CHECK_CUDA(cudaStreamCreate(&stream_));
    // CHECK_NVJPEG(nvjpegCreateEx(NVJPEG_BACKEND_DEFAULT, NULL, NULL, NVJPEG_FLAGS_DEFAULT, &handle_))
    CHECK_NVJPEG(nvjpegCreateSimple(&handle_));
    CHECK_NVJPEG(nvjpegEncoderStateCreate(handle_, &state_, stream_));
    CHECK_NVJPEG(nvjpegEncoderParamsCreate(handle_, &params_, stream_));

    nvjpegEncoderParamsSetSamplingFactors(params_, NVJPEG_CSS_420, stream_);

    std::memset(&nv_image_, 0, sizeof(nv_image_));
}

NVJPEGCompressor::~NVJPEGCompressor() {
    CHECK_NVJPEG(nvjpegEncoderParamsDestroy(params_));
    CHECK_NVJPEG(nvjpegEncoderStateDestroy(state_));
    CHECK_NVJPEG(nvjpegDestroy(handle_));
    CHECK_CUDA(cudaStreamDestroy(stream_));
}

CompressedImagePtr NVJPEGCompressor::compress(const Image &msg, int quality, ImageFormat format) {
    #warning TODO: implement format conversion or get rid of the parameter
    CompressedImagePtr compressed_msg = std::make_shared<CompressedImage>();
    compressed_msg->header = msg.header;
    compressed_msg->format = "jpeg";

    nvjpegEncoderParamsSetQuality(params_, quality, stream_);

    setNVImage(msg);
    CHECK_NVJPEG(nvjpegEncodeImage(handle_, state_, params_, &nv_image_, NVJPEG_INPUT_RGBI,
                                   msg.width, msg.height, stream_));
    
    unsigned long out_buf_size = 0;

    CHECK_NVJPEG(nvjpegEncodeRetrieveBitstream(handle_, state_, NULL, &out_buf_size, stream_));
    compressed_msg->data.resize(out_buf_size);
    CHECK_NVJPEG(nvjpegEncodeRetrieveBitstream(handle_, state_, compressed_msg->data.data(),
                                               &out_buf_size, stream_));

    CHECK_CUDA(cudaStreamSynchronize(stream_));

    return compressed_msg;
}

void NVJPEGCompressor::setNVImage(const Image &msg) {
    unsigned char *p = nullptr;
    CHECK_CUDA(cudaMallocAsync((void **)&p, msg.data.size(), stream_));
    if (nv_image_.channel[0] != NULL) {
        CHECK_CUDA(cudaFreeAsync(nv_image_.channel[0], stream_));
    }

    CHECK_CUDA(cudaMemcpyAsync(p, msg.data.data(), msg.data.size(), cudaMemcpyHostToDevice, stream_));

    // int channels = image.size() / (image.width * image.height);
    int channels = 3;

    std::memset(&nv_image_, 0, sizeof(nv_image_));

    // Assuming RGBI/BGRI
    nv_image_.pitch[0] = msg.width * channels;
    nv_image_.channel[0] = p;
}
#endif

} // namespace JpegCompressor