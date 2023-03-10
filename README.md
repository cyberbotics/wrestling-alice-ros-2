# Humanoid Robot Wrestling Controller Example

[![webots.cloud - Competition](https://img.shields.io/badge/webots.cloud-Competition-007ACC)][1]

## Alice ROS 2 controller

Minimalist ROS 2 package controller example for the [Humanoid Robot Wrestling Competition](https://github.com/cyberbotics/wrestling).
This basic package control motors using the Webots API provided by the [webots_ros2_driver](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_driver) package. An example `/IMU` topic has been implemented in [webots_controller.urdf](./controllers/participant/resource/webots_controller.urdf) following [this guide](https://github.com/cyberbotics/webots_ros2/wiki/References-Devices).

The Docker image used in the competition is a lightweight humble image that does not have colcon installed so we pre-build the package using the [build_controller.sh](./controllers/build_controller.sh) script.

Here is the [nao_controller.py](./controllers/participant/participant/nao_controller.py) file using Webots' [Robot API](https://cyberbotics.com/doc/reference/robot):

``` Python
import rclpy

# This is a workaround so that Webots' Python controller classes can be used
# in this case, we need it to import the Motion class
import os
from ament_index_python.packages import get_package_prefix
os.environ['WEBOTS_HOME'] = get_package_prefix('webots_ros2_driver')
from controller import Motion


class NaoDriver:
    def init(self, webots_node, properties):
        # we play on loop the HandWave motion
        self.handWave = Motion('../motions/HandWave.motion')
        self.handWave.setLoop(True)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('nao_driver')

    def step(self):
        # Mandatory function to go to the next simulation step
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.handWave.play()
```

And here is the [ROS 2 launch file](./controllers/participant/launch/robot_launch.py) using [webots_ros2_driver](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_driver):

``` Python
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
```

Here is the [Dockerfile](./controllers/Dockerfile) used by the controller:

``` Dockerfile
# We use the eprosima/vulcanexus:humble-simulation image because it is light
# It has ROS2 and webots_ros2 installed
FROM eprosima/vulcanexus:humble-simulation-2.1.1

WORKDIR /usr/local/webots-project/controllers/participant

# Copies all the files of the controllers folder into the docker container
RUN mkdir -p /usr/local/webots-project/controllers
COPY . /usr/local/webots-project/controllers

# The eprosima/vulcanexus:humble-simulation Docker image does not have colcon installed
# We install it and build the participant package
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build

# Environment variable needed to connect to Webots instance
ARG WEBOTS_CONTROLLER_URL
ENV WEBOTS_CONTROLLER_URL=${WEBOTS_CONTROLLER_URL}

# Source the ROS humble setup file and run the participant package
CMD . /opt/ros/humble/setup.sh && . /usr/local/webots-project/controllers/participant/install/setup.sh && ros2 launch participant robot_launch.py
```

[Bob](https://github.com/cyberbotics/wrestling-bob-ros-2) is a more advanced ROS 2 robot controller able to win against Alice.

[1]: https://webots.cloud/run?version=R2022b&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwrestling%2Fblob%2Fmain%2Fworlds%2Fwrestling.wbt&type=competition "Leaderboard"
