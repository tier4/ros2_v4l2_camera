# v4l2_camera

A ROS 2 camera driver using Video4Linux2 (V4L2).

## Installation
[This article](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304) details how to build and run this package. It focuses on Raspberry Pi OS with the Raspberry Pi Camera Module V2 but should generalise for most systems.

### Building from source
If you need to modify the code or ensure you have the latest update you will need to clone this repo then build the package.

    $ git clone --branch galactic https://github.com/tier4/ros2_v4l2_camera.git
    $ colcon build

Most users will also want to set up compressed transport using the dependencies below.

### Usage
Publish camera images, using the default parameters:

        ros2 run v4l2_camera v4l2_camera_node

Preview the image (open another terminal):

        ros2 run rqt_image_view rqt_image_view

## Dependencies

* `image_transport` - makes it possible to set up compressed transport
  of the images, as described below.

    The ROS 2 port of `image_transport` in the `image_common`
    repository is needed inside of your workspace:

        git clone --branch ros2 https://github.com/ros-perception/image_common.git src/image_common

    Note that `image_transport` only supports raw transport by default
    and needs additional plugins to actually provide compression; see
    below how to do this.

## Nodes

### v4l2_camera_node

The `v4l2_camera_node` interfaces with standard V4L2 devices and
publishes images as `sensor_msgs/Image` messages.

#### Published Topics

* `/image_raw` - `sensor_msgs/Image`

    The image.

#### Parameters

* `video_device` - `string`, default: `"/dev/video0"`

    The device the camera is on.

* `pixel_format` - `string`, default: `"YUYV"`

    The pixel format to request from the camera. Must be a valid four
    character '[FOURCC](http://fourcc.org/)' code [supported by
    V4L2](https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/videodev.html)
    and by your camera. The node outputs the available formats
    supported by your camera when started.  
    Currently supported: `"YUYV"`, `"UYVY"` or `"GREY"`. (`"UYVY"` support is only available on systems with CUDA)

* `output_encoding` - `string`, default: `"rgb8"`

    The encoding to use for the output image.  
    Currently supported: `"rgb8"`, `"yuv422"` or `"mono8"`.

* `image_size` - `integer_array`, default: `[640, 480]`

    Width and height of the image.

* `time_per_frame` - `integer_array`, default: current device setting

    The time between two successive frames. The expected value is a
    ratio defined by an array of 2 integers. For instance, a value of
    `[1, 30]` sets a period of 1/30, and thus a framrate of 30Hz.

    If the provided period is not supported, the driver may choose
    another period near to it. In that case the parameter change is
    reported to have failed.

* `use_v4l2_buffer_timestamps` - `bool`, default: `true`

    Flag to determine image timestamp behaviour. When `true`, the images
    will be timestamped according to the V4L2 buffer timestamps. When
    `false` the image timestamps will be the system time when the image
    buffer is read.

* `timestamp_offset` - `int64_t`, default: `0`

    Offset to be added to the image timestamp, in nanoseconds. This is
    useful to correct for delays in the image capture pipeline, when
    performing synchronization with other sensor data. Note that this 
    value will usually be negative (correcting for delays rather than 
    adding delay to the timestamp).

* Camera Control Parameters

    Camera controls, such as brightness, contrast, white balance, etc,
    are automatically made available as parameters. The driver node
    enumerates all controls, and creates a parameter for each, with
    the corresponding value type. The parameter name is derived from
    the control name reported by the camera driver, made lower case,
    commas removed, and spaces replaced by underscores. So
    `Brightness` becomes `brightness`, and `White Balance, Automatic`
    becomes `white_balance_automatic`.

## Compressed Transport

By default `image_transport` only supports raw transfer, plugins are
required to enable compression. Standard ones are available in the
[`image_transport_plugins`](https://github.com/ros-perception/image_transport_plugins)
repository. These depend on the OpenCV facilities provided by the
`vision_opencv` repository. You can clone these into your workspace to
get these:

    cd path/to/workspace
    git clone https://github.com/ros-perception/vision_opencv.git --branch ros2 src/vision_opencv
    git clone https://github.com/ros-perception/image_transport_plugins.git --branch ros2 src/image_transport_plugins

### Building: Ubuntu

The following packages are required to be able to build the plugins:

    sudo apt install libtheora-dev libogg-dev libboost-python-dev

### Building: Arch

To get the plugins compiled on Arch Linux, a few special steps are
needed:

* Arch provides OpenCV 4.x, but OpenCV 3.x is required
* Arch provides VTK 8.2, but VTK 8.1 is required
* `boost-python` is used, which [needs to be linked to python libs
  explicitly](https://bugs.archlinux.org/task/55798):

        colcon build --symlink-install --packages-select cv_bridge --cmake-args "-DCMAKE_CXX_STANDARD_LIBRARIES=-lpython3.7m"

### Usage

If the compression plugins are compiled and installed in the current
workspace, they will be automatically used by the driver and an
additional `/image_raw/compressed` topic will be available.

Neither Rviz2 or `showimage` use `image_transport` (yet). Therefore, to
be able to view the compressed topic, it needs to be republished
uncompressed. `image_transport` comes with the `republish` node to do
this:

    ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed

The parameters mean:

* `compressed` - the transport to use for input, in this case
  'compressed'. Alternative: `raw`, to republish the raw `/image_raw`
  topic
* `in/compressed:=image_raw/compressed` - by default, `republish` uses
  the topics `in` and `out`, or `in/compressed` for example if the
  input transport is 'compressed'. This parameter is a ROS remapping
  rule to map those names to the actual topic to use.
* `raw` - the transport to use for output. If omitted, all available
  transports are provided.
* `out:=image_raw/uncompressed` - remapping of the output topic.
