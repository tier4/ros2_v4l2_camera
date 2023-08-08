cmake_minimum_required(VERSION 3.5)
project(v4l2_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  image_transport
  nodelet
  pluginlib
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    camera_info_manager
    image_transport
    nodelet
    pluginlib
    roscpp
    sensor_msgs
  LIBRARIES
    camera_info_manager
    image_transport
    nodelet
    pluginlib
    roscpp
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(CUDA)

if (CUDA_FOUND)
  message(STATUS "CUDA Support is enabled in ${PROJECT_NAME}")
  cuda_add_library(v4l2_camera
    src/v4l2_camera.cpp
    src/v4l2_camera_device.cpp
    src/v4l2_camera_nodelet.cpp
  )
  target_compile_definitions(v4l2_camera PUBLIC
    ENABLE_CUDA
  )
  target_include_directories(v4l2_camera PUBLIC
    ${CUDA_INCLUDE_DIR}
  )
  target_link_libraries(v4l2_camera
    ${CUDA_nppicc_LIBRARY}
    ${CUDA_nppisu_LIBRARY}
  )
else()
  add_library(v4l2_camera
    src/v4l2_camera.cpp
    src/v4l2_camera_device.cpp
    src/v4l2_camera_nodelet.cpp
    )
endif()

target_include_directories(v4l2_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_options(v4l2_camera PRIVATE -Werror)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(v4l2_camera PRIVATE "V4L2_CAMERA_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS v4l2_camera
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(v4l2_camera_node src/v4l2_camera_node.cpp)
target_include_directories(v4l2_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(v4l2_camera_node
  v4l2_camera
  ${catkin_LIBRARIES}
  )
target_compile_options(v4l2_camera_node PRIVATE -Werror)

# add_executable(v4l2_camera_compose_test src/v4l2_camera_compose_test.cpp)
# target_include_directories(v4l2_camera_compose_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(v4l2_camera_compose_test v4l2_camera)
# target_compile_options(v4l2_camera_compose_test PRIVATE -Werror)

# install(TARGETS v4l2_camera_node v4l2_camera_compose_test
#   EXPORT export_${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS v4l2_camera_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
