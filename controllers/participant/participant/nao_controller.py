import rclpy


class NaoDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # we initialize the shoulder pitch motors using the Robot.getDevice() function:
        self.__RShoulderPitch = self.__robot.getDevice("RShoulderPitch")
        self.__LShoulderPitch = self.__robot.getDevice("LShoulderPitch")
        self.__RShoulderRoll = self.__robot.getDevice("RShoulderRoll")

        # to control a motor, we use the setPosition() function:
        self.__RShoulderPitch.setPosition(-1.3)
        self.__LShoulderPitch.setPosition(1.3)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('nao_driver')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        if self.__robot.getTime() % 2 < 1:
            self.__RShoulderRoll.setPosition(0)
        else:
            self.__RShoulderRoll.setPosition(-0.3)
