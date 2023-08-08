cmake_minimum_required(VERSION 3.5)
project(gpu_imgproc LANGUAGES CUDA CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(ament_cmake_auto REQUIRED)
find_package (LibJpegTurbo)

if (${LibJpegTurbo_FOUND})
  message("LibJpegTurbo found")
  add_definitions(-DTURBOJPEG_AVAILABLE)
endif ()

ament_auto_find_build_dependencies()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()


find_package(CUDA)
find_package(NVJPEG)
find_library(CUDART_LIBRARY cudart ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
find_library(CULIBOS culibos ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
if (${NVJPEG_FOUND} AND CUDART_LIBRARY AND CULIBOS)
  add_definitions(-DNVJPEG_AVAILABLE)
else()
  message("NVJPEG or its dependencies not found")
endif()

find_library(CUDA_nppicc_LIBRARY nppicc ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
find_library(CUDA_nppidei_LIBRARY nppidei ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
find_library(CUDA_nppig_LIBRARY nppig ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
find_library(CUDA_nppisu_LIBRARY nppisu ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})

if (CUDA_nppicc_LIBRARY AND CUDA_nppidei_LIBRARY AND CUDA_nppig_LIBRARY AND CUDA_nppisu_LIBRARY)
  message("NPP found")
  add_definitions(-DNPP_AVAILABLE)
endif()

find_package(OpenCV REQUIRED PATHS "/usr/local/") # what to do about this one?
# If OpenCV is found
if (${OpenCV_FOUND})
  message("OpenCV version: ${OpenCV_VERSION}")
  find_package(cv_bridge REQUIRED)
  add_definitions(-DOPENCV_AVAILABLE)
endif()

if (OpenCV_CUDA_VERSION)
  # Found OpenCV with CUDA support
  message("OpenCV CUDA version: ${OpenCV_CUDA_VERSION}")
  add_definitions(-DOPENCV_CUDA_AVAILABLE)
else()
  message("OpenCV CUDA not found")
endif()

if (EXISTS "/etc/nv_tegra_release")
  set(JETSON TRUE)
  message(STATUS "Jetson platform detected")
  add_definitions(-DJETSON_AVAILABLE)
else ()
  set(JETSON FALSE)
  message(STATUS "Non-Jetson platform detected")
endif()

if (NOT JETSON AND NOT NVJPEG_FOUND AND NOT LibJpegTurbo_FOUND)
  message(FATAL_ERROR "No JPEG encoder found")
endif()

find_package(sensor_msgs REQUIRED)

# Jetson
if (JETSON)
  include(FetchContent)
  FetchContent_Declare(cuda-api-wrappers
      GIT_REPOSITORY https://github.com/eyalroz/cuda-api-wrappers.git
      GIT_TAG 831666a0bfd1af0f44f4fa234ee2d983d347fcaa # v0.6.1-rc1
  )
  FetchContent_MakeAvailable(cuda-api-wrappers)

  cuda_add_library(color_space SHARED
    src/accelerator/color_space.cu
  )

  ament_auto_add_library(jpeg_compressor SHARED
    src/accelerator/jpeg_compressor.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
  )
  target_include_directories(jpeg_compressor PUBLIC
    ${CUDA_INCLUDE_DIR}
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${SYS_ROOT}/usr/src/jetson_multimedia_api/include
    ${SYS_ROOT}/usr/src/jetson_multimedia_api/include/libjpeg-8b
  )
  target_link_directories(jpeg_compressor PUBLIC
    ${SYS_ROOT}/lib/aarch64-linux-gnu
    ${SYS_ROOT}/usr/lib/aarch64-linux-gnu
    ${SYS_ROOT}/usr/lib/aarch64-linux-gnu/tegra)
  target_link_libraries(jpeg_compressor
    ${CUDA_nppicc_LIBRARY}
    ${LIBJPEGTURBO_LIBRARIES}
    nvjpeg
    cuda-api-wrappers::runtime-and-driver
    color_space
  )

  message(STATUS "Using Jetson Multimedia API")

  # include_directories(/usr/local/include/opencv4)
  ament_auto_add_library(rectifier SHARED
    src/accelerator/rectifier.cpp)
  target_include_directories(rectifier PUBLIC
    ${CUDA_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_directories(rectifier PUBLIC
    ${SYS_ROOT}/lib/aarch64-linux-gnu
    ${SYS_ROOT}/usr/lib/aarch64-linux-gnu
    ${SYS_ROOT}/usr/lib/aarch64-linux-gnu/tegra)
  target_link_libraries(rectifier
    ${CUDA_nppidei_LIBRARY}
    ${CUDA_nppig_LIBRARY}
    ${CUDA_nppicc_LIBRARY}
    ${CUDA_nppisu_LIBRARY}
    ${OpenCV_LIBRARIES}
  )

  include_directories(
          include
          ${CUDA_INCLUDE_DIRS}
          ${OpenCV_INCLUDE_DIRS}
  )

  ament_auto_add_library(gpu_imgproc SHARED
          src/gpu_imgproc.cpp
          )

  target_link_libraries(gpu_imgproc
          ${CUDA_LIBRARIES}
          ${OpenCV_LIBRARIES}
          rectifier
          jpeg_compressor
          )

  rclcpp_components_register_node(gpu_imgproc
          PLUGIN "GpuImgProc"
          EXECUTABLE gpu_imgproc_node
          )

  install(
          TARGETS gpu_imgproc
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)
else()
  ament_auto_add_library(jpeg_compressor SHARED
    src/accelerator/jpeg_compressor.cpp
  )
  target_include_directories(jpeg_compressor PUBLIC
    ${CUDA_INCLUDE_DIR}
    ${CUDA_NPP_INCLUDES}
    ${LibJpegTurbo_INCLUDE_DIRS}
    ${NVJPEG_INCLUDE_DIRS}
    # ${CUDART_INCLUDE_DIR}
    # ${CULIBOS_INCLUDE_DIR}
  )
  target_link_libraries(jpeg_compressor
    ${CUDA_nppicc_LIBRARY}
    ${LibJpegTurbo_LIBRARY}
    ${NVJPEG_LIBRARY}
    ${CUDART_LIBRARY}
    ${CULIBOS}
    # nvjpeg
  )

  ament_auto_add_library(rectifier SHARED
    src/accelerator/rectifier.cpp)
  target_include_directories(rectifier PUBLIC
    ${CUDA_INCLUDE_DIR}
    ${CUDA_NPP_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(rectifier
    ${CUDA_nppidei_LIBRARY}
    ${CUDA_nppig_LIBRARY}
    ${CUDA_nppicc_LIBRARY}
    ${CUDA_nppisu_LIBRARY}
    ${OpenCV_LIBRARIES}
  )

  include_directories(
          include
          ${CUDA_INCLUDE_DIRS}
          ${OpenCV_INCLUDE_DIRS}
  )

  ament_auto_add_library(gpu_imgproc SHARED
          src/gpu_imgproc.cpp
          )

  target_link_libraries(gpu_imgproc
          ${CUDA_LIBRARIES}
          ${OpenCV_LIBRARIES}
          rectifier
          jpeg_compressor
          )

  rclcpp_components_register_node(gpu_imgproc
          PLUGIN "GpuImgProc"
          EXECUTABLE gpu_imgproc_node
          )

  install(
          TARGETS gpu_imgproc
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)