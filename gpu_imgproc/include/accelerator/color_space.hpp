#ifndef JETSON_ENCODER_COMPRESSED_IMAGE_TRANSPORT_RGB8_TO_BGR8
#define JETSON_ENCODER_COMPRESSED_IMAGE_TRANSPORT_RGB8_TO_BGR8

#include <cuda_runtime_api.h>

#include <cstdint>

// namespace jetson_encoder_compressed_image_transport
// {
typedef enum ColorSpaceStandard {
    ColorSpaceStandard_BT709 = 1,
    ColorSpaceStandard_Unspecified = 2,
    ColorSpaceStandard_Reserved = 3,
    ColorSpaceStandard_FCC = 4,
    ColorSpaceStandard_BT470 = 5,
    ColorSpaceStandard_BT601 = 6,
    ColorSpaceStandard_SMPTE240M = 7,
    ColorSpaceStandard_YCgCo = 8,
    ColorSpaceStandard_BT2020 = 9,
    ColorSpaceStandard_BT2020C = 10
} ColorSpaceStandard;

cudaError_t cudaRGB8ToBGR8(uint8_t * input, int width, int height, int step);

cudaError_t cudaBGRToYUV420(uint8_t * input, uint8_t * output, int width, int height, int matrix = 0);
cudaError_t cudaRGBToYUV420(uint8_t * input, uint8_t * output, int width, int height, int matrix = 0);
// } // namespace jetson_encoder_compressed_image_transport

#endif  // JETSON_ENCODER_COMPRESSED_IMAGE_TRANSPORT_RGB8_TO_BGR8