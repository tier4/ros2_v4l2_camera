^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package v4l2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2020-11-07)
------------------
* Handle non discrete frame sizes; fixes support for Raspberry Pi + Camera Module
* Contributors: Sander G. van Dijk

0.3.0 (2020-09-26)
------------------
* Publishing is done on private topics to enable remapping of the namespace
* CameraInfo is published in intra-process communication mode
* Added parameter descriptions
* Contributors: Christian Rauch, Marcus M. Scheunemann, Sander G. van Dijk

0.2.1 (2020-08-06)
------------------
* Hold reference to parameters callback handle
* Contributors: Jacob Perron

0.2.0 (2020-06-13)
------------------
* Set frame_id on published images, default to "camera"
* Output FOURCC code of available formats
* Add parameter for setting the pixel format
  Default to using YUYV
* Contributors: Sander G. van Dijk

0.1.1 (2019-08-12)
------------------
* Add missing rclcpp_components build dependency
* Contributors: Sander G. van Dijk

0.1.0 (2019-08-11)
------------------
