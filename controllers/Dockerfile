FROM eprosima/vulcanexus:humble-simulation
# This ros 2 controller package was built using the image eprosima/vulcanexus:humble-desktop
# We use the humble-simulation image because it is lighter and we don't need building tools anymore

# Fix needed until vulcanexus uses webots-ros2 version 2023.0.1
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/controller:$LD_LIBRARY_PATH

# Copies all the files of the controllers folder into the docker container
RUN mkdir -p /usr/local/webots-project/controllers
COPY . /usr/local/webots-project/controllers

# Environment variable needed to connect to Webots instance
ARG WEBOTS_CONTROLLER_URL
ENV WEBOTS_CONTROLLER_URL=${WEBOTS_CONTROLLER_URL}

# Source the ROS humble setup file
CMD . /opt/ros/humble/setup.sh && . /usr/local/webots-project/controllers/participant/install/setup.sh && ros2 launch participant robot_launch.py
