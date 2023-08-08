// Copyright 2022 Daisuke Nishimatsu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "accelerator/color_space.hpp"

// namespace jetson_encoder_compressed_image_transport
// {
inline __device__ __host__ int iDivUp(int a, int b) {return (a % b != 0) ? (a / b + 1) : (a / b);}

__global__ void RGB8ToBGR8(uint8_t * input, int width, int height, int step)
{
  //2D Index of current thread
  const int x_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int y_index = blockIdx.y * blockDim.y + threadIdx.y;

  //Only valid threads perform memory I/O
  if ((x_index < width) && (y_index < height)) {
    //Location of colored pixel in input
    const int color_tid = y_index * step + (3 * x_index);
    const unsigned char t = input[color_tid + 0];
    input[color_tid + 0] = input[color_tid + 2];
    input[color_tid + 2] = t;
  }
}

cudaError_t cudaRGB8ToBGR8(uint8_t * input, int width, int height, int step)
{
  if (!input) {
    return cudaErrorInvalidDevicePointer;
  }

  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));

  RGB8ToBGR8 << < gridDim, blockDim >> > (input, width, height, step);

  return cudaGetLastError();
}

__constant__ float matRGB2YUV[3][3];


inline void getConstants(int matrix, float & wr, float & wb, int & black, int & white, int & max)
{
  black = 16;
  white = 235;
  max = 255;

  switch (matrix) {
    case ColorSpaceStandard_BT709:
    default:
      wr = 0.2126f; wb = 0.0722f;
      break;

    case ColorSpaceStandard_FCC:
      wr = 0.30f; wb = 0.11f;
      break;

    case ColorSpaceStandard_BT470:
    case ColorSpaceStandard_BT601:
      wr = 0.2990f; wb = 0.1140f;
      break;

    case ColorSpaceStandard_SMPTE240M:
      wr = 0.212f; wb = 0.087f;
      break;

    case ColorSpaceStandard_BT2020:
    case ColorSpaceStandard_BT2020C:
      wr = 0.2627f; wb = 0.0593f;
      // 10-bit only
      black = 64 << 6; white = 940 << 6;
      max = (1 << 16) - 1;
      break;
  }
}

void setMatRGB2YUV(int matrix)
{
  float wr, wb;
  int black, white, max;
  getConstants(matrix, wr, wb, black, white, max);
  float mat[3][3] = {
    wr, 1.0f - wb - wr, wb,
    -0.5f * wr / (1.0f - wb), -0.5f * (1 - wb - wr) / (1.0f - wb), 0.5f,
    0.5f, -0.5f * (1.0f - wb - wr) / (1.0f - wr), -0.5f * wb / (1.0f - wr),
  };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat[i][j] = (float)(1.0 * (white - black) / max * mat[i][j]);
    }
  }
  cudaMemcpyToSymbol(matRGB2YUV, mat, sizeof(mat));
}

template<class YUVUnit, class RGBUnit>
__device__ inline YUVUnit RGBToY(RGBUnit r, RGBUnit g, RGBUnit b)
{
  const YUVUnit low = 1 << (sizeof(YUVUnit) * 8 - 4);
  return matRGB2YUV[0][0] * r + matRGB2YUV[0][1] * g + matRGB2YUV[0][2] * b + low;
}

template<class YUVUnit, class RGBUnit>
__device__ inline YUVUnit RGBToU(RGBUnit r, RGBUnit g, RGBUnit b)
{
  const YUVUnit mid = 1 << (sizeof(YUVUnit) * 8 - 1);
  return matRGB2YUV[1][0] * r + matRGB2YUV[1][1] * g + matRGB2YUV[1][2] * b + mid;
}

template<class YUVUnit, class RGBUnit>
__device__ inline YUVUnit RGBToV(RGBUnit r, RGBUnit g, RGBUnit b)
{
  const YUVUnit mid = 1 << (sizeof(YUVUnit) * 8 - 1);
  return matRGB2YUV[2][0] * r + matRGB2YUV[2][1] * g + matRGB2YUV[2][2] * b + mid;
}

template<class YUVUnitx2>
__global__ static void BGRToYUVKernel(
  uint8_t * input, uint8_t * output, int yuv_pitch, int width, int height)
{
  int x = (threadIdx.x + blockIdx.x * blockDim.x) * 2;
  int y = (threadIdx.y + blockIdx.y * blockDim.y) * 2;
  if (x + 1 >= width || y + 1 >= height) {
    return;
  }

  uint8_t * src = input + x * 3 + y * width * 3;

  uint8_t * int2a = src;
  uint8_t * int2b = src + width * 3;

  uint8_t b = (int2a[0] + int2a[3] + int2b[0] + int2b[3]) / 4,
    g = (int2a[1] + int2a[4] + int2b[1] + int2b[4]) / 4,
    r = (int2a[2] + int2a[5] + int2b[2] + int2b[5]) / 4;

  uint8_t * dst = output + x + y * width;

  dst[0] = RGBToY<uint8_t, uint8_t>(int2a[0 + 2], int2a[0 + 1], int2a[0 + 0]);
  dst[1] = RGBToY<uint8_t, uint8_t>(int2a[1 * 3 + 2], int2a[1 * 3 + 1], int2a[1 * 3 + 0]);
  dst[width] = RGBToY<uint8_t, uint8_t>(int2b[0 + 2], int2b[0 + 1], int2b[0 + 0]);
  dst[width + 1] = RGBToY<uint8_t, uint8_t>(int2b[1 * 3 + 2], int2b[1 * 3 + 1], int2b[1 * 3 + 0]);
  *(output + width * height +
  static_cast<size_t>(width / 2) * (static_cast<size_t>(y / 2)) + x / 2) =
    RGBToU<uint8_t, uint8_t>(r, g, b);
  *(output + width * height + static_cast<size_t>(width / 2) * static_cast<size_t>(height / 2) +
  static_cast<size_t>(width / 2) *
  (static_cast<size_t>(y / 2)) + x / 2) = RGBToV<uint8_t, uint8_t>(r, g, b);
}

template<class YUVUnitx2>
__global__ static void RGBToYUVKernel(
  uint8_t * input, uint8_t * output, int yuv_pitch, int width, int height)
{
  int x = (threadIdx.x + blockIdx.x * blockDim.x) * 2;
  int y = (threadIdx.y + blockIdx.y * blockDim.y) * 2;
  if (x + 1 >= width || y + 1 >= height) {
    return;
  }

  uint8_t * src = input + x * 3 + y * width * 3;

  uint8_t * int2a = src;
  uint8_t * int2b = src + width * 3;

  uint8_t r = (int2a[0] + int2a[3] + int2b[0] + int2b[3]) / 4,
    g = (int2a[1] + int2a[4] + int2b[1] + int2b[4]) / 4,
    b = (int2a[2] + int2a[5] + int2b[2] + int2b[5]) / 4;

  uint8_t * dst = output + x + y * width;

  dst[0] = RGBToY<uint8_t, uint8_t>(int2a[0 + 0], int2a[0 + 1], int2a[0 + 2]);
  dst[1] = RGBToY<uint8_t, uint8_t>(int2a[1 * 3 + 0], int2a[1 * 3 + 1], int2a[1 * 3 + 2]);
  dst[width] = RGBToY<uint8_t, uint8_t>(int2b[0 + 0], int2b[0 + 1], int2b[0 + 2]);
  dst[width + 1] = RGBToY<uint8_t, uint8_t>(int2b[1 * 3 + 0], int2b[1 * 3 + 1], int2b[1 * 3 + 2]);
  *(output + width * height +
  static_cast<size_t>(width / 2) * (static_cast<size_t>(y / 2)) + x / 2) =
    RGBToU<uint8_t, uint8_t>(r, g, b);
  *(output + width * height + static_cast<size_t>(width / 2) * static_cast<size_t>(height / 2) +
  static_cast<size_t>(width / 2) *
  (static_cast<size_t>(y / 2)) + x / 2) = RGBToV<uint8_t, uint8_t>(r, g, b);
}

cudaError_t cudaBGRToYUV420(uint8_t * input, uint8_t * output, int width, int height, int matrix)
{
  if (!input) {
    return cudaErrorInvalidDevicePointer;
  }

  setMatRGB2YUV(matrix);
  BGRToYUVKernel<ushort2>
    << < dim3((width + 63) / 32 / 2, (height + 3) / 2 / 2), dim3(32, 2) >> >
    (input, output, 4 * width, width, height);

  return cudaGetLastError();
}

cudaError_t cudaRGBToYUV420(uint8_t * input, uint8_t * output, int width, int height, int matrix)
{
  if (!input) {
    return cudaErrorInvalidDevicePointer;
  }

  setMatRGB2YUV(matrix);
  RGBToYUVKernel<ushort2>
    << < dim3((width + 63) / 32 / 2, (height + 3) / 2 / 2), dim3(32, 2) >> >
    (input, output, 4 * width, width, height);

  return cudaGetLastError();
}

// }  // namespace jetson_encoder_compressed_image_transport