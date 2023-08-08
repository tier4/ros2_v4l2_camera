cmake_minimum_required(VERSION 3.5)
project(gpu_imgproc LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gpu_imgproc sensor_msgs
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_geometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package (LibJpegTurbo)
if (${LibJpegTurbo_FOUND})
  message("LibJpegTurbo found")
  add_definitions(-DTURBOJPEG_AVAILABLE)
endif ()

find_package(CUDA)
if (CUDA_FOUND)
  enable_language(CUDA)
endif()
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
  set(NPP_FOUND TRUE)
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
  set_target_properties(color_space PROPERTIES CUDA_ARCHITECTURES 72)

  add_library(jpeg_compressor SHARED
    src/accelerator/jpeg_compressor.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
    /usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
  )
  target_include_directories(jpeg_compressor PUBLIC
    ${CUDA_INCLUDE_DIRS}
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
  add_library(rectifier SHARED
    src/accelerator/rectifier.cpp)
  target_include_directories(rectifier PUBLIC
    ${CUDA_INCLUDE_DIRS}
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
    ${catkin_LIBRARIES}
  )

  add_library(gpu_imgproc SHARED
          src/gpu_imgproc.cpp
          )

  target_link_libraries(gpu_imgproc
          ${CUDA_LIBRARIES}
          ${OpenCV_LIBRARIES}
          rectifier
          jpeg_compressor
          ${catkin_LIBRARIES}
          )

  add_executable(gpu_imgproc_node
          src/gpu_imgproc_node.cpp
          )
  target_include_directories(gpu_imgproc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
  target_link_libraries(gpu_imgproc_node
          ${CUDA_LIBRARIES}
          ${OpenCV_LIBRARIES}
          ${catkin_LIBRARIES}
          gpu_imgproc
          )

  install(
          TARGETS gpu_imgproc_node gpu_imgproc color_space
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)
else()
  add_library(jpeg_compressor SHARED
    src/accelerator/jpeg_compressor.cpp
  )

  if (NVJPEG_FOUND AND LibJpegTurbo_FOUND)
    target_include_directories(jpeg_compressor PUBLIC
      ${CUDA_INCLUDE_DIRS}
      ${LibJpegTurbo_INCLUDE_DIRS}
      ${NVJPEG_INCLUDE_DIRS}
    )
    target_link_libraries(jpeg_compressor
      ${CUDA_nppicc_LIBRARY}
      ${LibJpegTurbo_LIBRARY}
      ${NVJPEG_LIBRARY}
      ${CUDART_LIBRARY}
      ${CULIBOS}
      ${catkin_LIBARIES}
    )
  elseif (NVJPEG_FOUND)
    target_include_directories(jpeg_compressor PUBLIC
      ${CUDA_INCLUDE_DIRS}
      ${NVJPEG_INCLUDE_DIRS}
    )
    target_link_libraries(jpeg_compressor
      ${CUDA_nppicc_LIBRARY}
      ${NVJPEG_LIBRARY}
      ${CUDART_LIBRARY}
      ${CULIBOS}
      ${catkin_LIBARIES}
    )
  elseif (LibJpegTurbo_FOUND)
    target_include_directories(jpeg_compressor PUBLIC
      ${CUDA_INCLUDE_DIRS}
      ${LibJpegTurbo_INCLUDE_DIRS}
    )
    target_link_libraries(jpeg_compressor
      ${LibJpegTurbo_LIBRARY}
      ${catkin_LIBARIES}
    )
  else()
    message(FATAL_ERROR "No JPEG encoder found")
  endif()

  add_library(rectifier SHARED
    src/accelerator/rectifier.cpp)

  if (NPP_FOUND AND OPENCV_FOUND)
    message("Using NPP")
    target_include_directories(rectifier PUBLIC
      ${CUDA_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(rectifier
      ${CUDA_nppidei_LIBRARY}
      ${CUDA_nppig_LIBRARY}
      ${CUDA_nppicc_LIBRARY}
      ${CUDA_nppisu_LIBRARY}
      ${OpenCV_LIBRARIES}
      ${catkin_LIBRARIES}
    )
  elseif(OPENCV_FOUND)
    message("Using OpenCV")
    target_include_directories(rectifier PUBLIC
      ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(rectifier
      ${OpenCV_LIBRARIES}
      ${catkin_LIBRARIES}
    )
  else()
    message(FATAL_ERROR "No rectifier found")
  endif()

  add_library(gpu_imgproc SHARED
          src/gpu_imgproc.cpp
          )

  target_link_libraries(gpu_imgproc
          rectifier
          jpeg_compressor
          ${catkin_LIBRARIES}
          )
  target_include_directories(gpu_imgproc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

  add_executable(gpu_imgproc_node
          src/gpu_imgproc_node.cpp
          )
  target_include_directories(gpu_imgproc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
  target_link_libraries(gpu_imgproc_node
          ${catkin_LIBRARIES}
          gpu_imgproc
          )
  # target_compile_options(gpu_imgproc_node PRIVATE -Wall -Wextra -Wpedantic -Wunused-function)

  install(
          TARGETS gpu_imgproc_node gpu_imgproc
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)
endif()