import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

    
def generate_launch_description():
    
    share_dir = get_package_share_directory('image_pub')
    
    path_to_parameters_launch_arg = DeclareLaunchArgument(
        "path_to_parameters",
        default_value=TextSubstitution(
            text=os.path.join(
                share_dir,
                "config/imx21983_raw_image_pub.yaml"
            )
        )
    )

    return LaunchDescription([
        path_to_parameters_launch_arg,

        Node(
            package='image_pub',
            executable='mono_image_pub',
            name='raw_image_publisher',
            output='screen',
            parameters=[
                LaunchConfiguration('path_to_parameters'),
            ]
        )
    ])