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
