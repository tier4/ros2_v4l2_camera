#pragma once

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <string>

#ifdef TURBOJPEG_AVAILABLE
#include <turbojpeg.h>
#endif

#ifdef JETSON_AVAILABLE
#include <NvJpegEncoder.h>
#include <cuda/api.hpp>
#endif

#ifdef NVJPEG_AVAILABLE
#include <nvjpeg.h>
#endif

class NvJPEGEncoder;

namespace JpegCompressor {
using Image = sensor_msgs::Image;
using CompressedImage = sensor_msgs::CompressedImage;
typedef std::shared_ptr<CompressedImage> CompressedImagePtr;

enum class ImageFormat {
    RGB,
    BGR
};

#ifdef TURBOJPEG_AVAILABLE
class CPUCompressor {
public:
    CPUCompressor();
    ~CPUCompressor();

    CompressedImagePtr compress(const Image &msg, int quality = 90, int format = TJPF_RGB, int sampling = TJ_420);
private:
    tjhandle handle_;
    unsigned char *jpegBuf_;
    unsigned long size_;
};
#endif

#ifdef JETSON_AVAILABLE
class JetsonCompressor {
public:
    JetsonCompressor(std::string name);
    ~JetsonCompressor();

    CompressedImagePtr compress(const Image &msg, int quality = 90, ImageFormat format = ImageFormat::RGB);
private:
    NvJPEGEncoder *encoder_;
    size_t image_size{};
    size_t yuv_size{};
    cuda::memory::device::unique_ptr<uint8_t[]> dev_image;
    cuda::memory::host::unique_ptr<uint8_t[]> host_yuv;
    cuda::memory::device::unique_ptr<uint8_t[]> dev_yuv;
};
#endif

#ifdef NVJPEG_AVAILABLE
class NVJPEGCompressor {
public:
    NVJPEGCompressor();
    ~NVJPEGCompressor();

    CompressedImagePtr compress(const Image &msg, int quality = 90, ImageFormat format = ImageFormat::RGB);
private:
    // void setNVJPEGParams(int quality, ImageFormat format);
    void setNVImage(const Image &msg);

    cudaStream_t stream_;
    nvjpegHandle_t handle_;
    nvjpegEncoderState_t state_;
    nvjpegEncoderParams_t params_;
    nvjpegInputFormat_t input_format_;
    nvjpegChromaSubsampling_t subsampling_;
    nvjpegImage_t nv_image_;
}; 
#endif

} // namespace JpegCompressor