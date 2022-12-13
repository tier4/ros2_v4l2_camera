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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(CUDA)

if (CUDA_FOUND)
  message(STATUS "CUDA Support is enabled in ${PROJECT_NAME}")
  cuda_add_library(v4l2_camera
    src/v4l2_camera.cpp
    src/v4l2_camera_device.cpp
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
    src/v4l2_camera_device.cpp)
endif()
rclcpp_components_register_nodes(v4l2_camera "v4l2_camera::V4L2Camera")

target_include_directories(v4l2_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(v4l2_camera
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "camera_info_manager")

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
target_link_libraries(v4l2_camera_node v4l2_camera)
target_compile_options(v4l2_camera_node PRIVATE -Werror)

add_executable(v4l2_camera_compose_test src/v4l2_camera_compose_test.cpp)
target_include_directories(v4l2_camera_compose_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(v4l2_camera_compose_test v4l2_camera)
target_compile_options(v4l2_camera_compose_test PRIVATE -Werror)

install(TARGETS v4l2_camera_node v4l2_camera_compose_test
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  v4l2_camera
)

ament_export_dependencies(
  "camera_info_manager"
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
