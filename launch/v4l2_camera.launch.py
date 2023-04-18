from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
import yaml

def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    composable_nodes = [
        ComposableNode(
            package="v4l2_camera",
            plugin="v4l2_camera::V4L2Camera",
            name=['v4l2_camera_', LaunchConfiguration("camera_name")],
            namespace=LaunchConfiguration("v4l2_camera_namespace"),
            remappings=[
                (
                    "image_raw",
                    [
                        LaunchConfiguration("camera_name"),
                        '/',
                        LaunchConfiguration("image_topic"),
                    ],
                ),
                (
                    "image_raw/compressed",
                    [
                        LaunchConfiguration("camera_name"),
                        '/',
                        LaunchConfiguration("image_topic"),
                        '/compressed',
                    ],
                ),
                (
                    "image_raw/compressedDepth",
                    [
                        LaunchConfiguration("camera_name"),
                        '/',
                        LaunchConfiguration("image_topic"),
                        '/compressedDepth',
                    ],
                ),
                (
                    "image_raw/theora",
                    [
                        LaunchConfiguration("camera_name"),
                        '/',
                        LaunchConfiguration("image_topic"),
                        '/theora',
                    ],
                ),
                (
                    "camera_info",
                    [
                        LaunchConfiguration("camera_name"),
                        '/camera_info'
                    ],
                ),
            ],
            parameters=[
                load_composable_node_param("v4l2_camera_param_path"),
                {
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    # If an existing container is not provided, start container and load nodes into it
    v4l2_camera_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name=['v4l2_camera_', LaunchConfiguration('camera_name'),  '_container'],
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    # If an existing container name is provided load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return [v4l2_camera_container, load_composable_nodes]

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value, description=description))

    add_launch_arg('container', '',
                   description='container name to load composable nodes into it. '
                   'If it is not specified, a new container will be created')
    add_launch_arg('image_topic',
                   description='image topic name to be published')
    add_launch_arg('camera_name',
                   description='prefix to be added to the head of topic name')
    add_launch_arg('v4l2_camera_namespace', '/sensing/camera',
                   description='namespace in which the nodes launched')
    add_launch_arg('v4l2_camera_param_path',
                   description='path to the yaml file that contains parameters for v4l2_camera node')
    add_launch_arg('camera_info_url',
                   description='url to the yaml file that contains camera\'s intrinsic paramters')
    add_launch_arg('use_intra_process', 'False',
                   description='flag to use ROS2 intra process')
    add_launch_arg('use_sensor_data_qos', 'False',
                   description='flag to use sensor data QoS. '
                   'If true, the reliability of image topic QoS will be BEST_EFFORT, '
                   'otherwise be RELIABLE')
    add_launch_arg('publish_rate', "-1.0",
                   description='publish frame number per second. value <= 0 means no limitation on publish rate')

    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
