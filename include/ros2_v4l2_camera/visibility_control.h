#ifndef ROS2_V4L2_CAMERA__VISIBILITY_CONTROL_H_
#define ROS2_V4L2_CAMERA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS2_V4L2_CAMERA_EXPORT __attribute__ ((dllexport))
    #define ROS2_V4L2_CAMERA_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS2_V4L2_CAMERA_EXPORT __declspec(dllexport)
    #define ROS2_V4L2_CAMERA_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS2_V4L2_CAMERA_BUILDING_LIBRARY
    #define ROS2_V4L2_CAMERA_PUBLIC ROS2_V4L2_CAMERA_EXPORT
  #else
    #define ROS2_V4L2_CAMERA_PUBLIC ROS2_V4L2_CAMERA_IMPORT
  #endif
  #define ROS2_V4L2_CAMERA_PUBLIC_TYPE ROS2_V4L2_CAMERA_PUBLIC
  #define ROS2_V4L2_CAMERA_LOCAL
#else
  #define ROS2_V4L2_CAMERA_EXPORT __attribute__ ((visibility("default")))
  #define ROS2_V4L2_CAMERA_IMPORT
  #if __GNUC__ >= 4
    #define ROS2_V4L2_CAMERA_PUBLIC __attribute__ ((visibility("default")))
    #define ROS2_V4L2_CAMERA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS2_V4L2_CAMERA_PUBLIC
    #define ROS2_V4L2_CAMERA_LOCAL
  #endif
  #define ROS2_V4L2_CAMERA_PUBLIC_TYPE
#endif

#endif  // ROS2_V4L2_CAMERA__VISIBILITY_CONTROL_H_
