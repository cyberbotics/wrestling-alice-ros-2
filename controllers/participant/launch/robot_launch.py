import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('participant')
    controller_url = os.environ.get('WEBOTS_CONTROLLER_URL')

    my_robot_driver = WebotsController(
        robot_name=controller_url,
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'webots_controller.urdf')},
        ]
    )

    return LaunchDescription([
        my_robot_driver,
    ])
