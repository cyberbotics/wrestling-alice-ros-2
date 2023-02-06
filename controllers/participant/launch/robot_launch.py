import os
import pathlib
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('participant')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_controller.urdf')).read_text()

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': os.environ['WEBOTS_CONTROLLER_URL']},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        my_robot_driver,
    ])
