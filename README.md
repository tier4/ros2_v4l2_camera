# ros2_v4l2_camera

A ROS 2 camera driver using Video4Linux2 (V4L2).

## Nodes

### ros2_v4l2_camera_node

The `ros2_v4l2_camera_node` interfaces with standard V4L2 devices and
publishes images as `sensor_msgs/Image` messages.

Uses the `image_transport` to allow compressed image transport. For
this, the ROS 2 port of `image_transport` in the `image_common`
repository is needed:

    cd src && git clone --branch ros2  https://github.com/ros-perception/image_common.git

TODO: research further and document how to actually get compression.

#### Published Topics

* `/raw_image` - `sensor_msgs/Image`

    The image.

#### Parameters

* `video_device` - `string`, default: `"/dev/video0"`

    The device the camera is on.

* `output_encoding` - `string`, default: `"rgb8"`

    The encoding to use for the output image. Currently supported: "rgb8" or "yuv422".

* `image_size` - `integer_array`, default: `[640, 480]`

    Width and height of the image.

* Camera Control Parameters

    Camera controls, such as brightness, contrast, white balance, etc,
    are automatically made available as parameters. The driver node
    enumerates all controls, and creates a parameter for each, with
    the corresponding value type. The parameter name is derived from
    the control name reported by the camera driver, made lower case,
    commas removed, and spaces replaced by underscorse. So
    `Brightness` becomes `brightness`, and `White Balance, Automatic`
    becomes `white_balance_automatic`.
