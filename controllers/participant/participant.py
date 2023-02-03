# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Minimalist controller example for the Robot Wrestling Tournament.
Demonstrates how to play a simple motion file.
"""

from controller import Robot, Motion


class Alice (Robot):
    def run(self):
        # motion files are text files containing pre-recorded positions of the robot's joints
        handWave = Motion('../motions/HandWave.motion')
        handWave.setLoop(True)
        handWave.play()
        # retrieves the simulation time step (ms) from the world file
        time_step = int(self.getBasicTimeStep())
        while self.step(time_step) != -1:  # Mandatory function to make the simulation run
            pass


# create the Robot instance and run main loop
wrestler = Alice()
wrestler.run()
