# We use the eprosima/vulcanexus:humble-simulation image because it is light
FROM eprosima/vulcanexus:humble-simulation

# Fix needed until vulcanexus uses webots-ros2 version 2023.0.1
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/controller:$LD_LIBRARY_PATH

WORKDIR /usr/local/webots-project/controllers/participant

# Copies all the files of the controllers folder into the docker container
RUN mkdir -p /usr/local/webots-project/controllers
COPY . /usr/local/webots-project/controllers

# The eprosima/vulcanexus:humble-simulation Docker image does not have colcon installed
# We install it and build the participant package
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    colcon build

# Environment variable needed to connect to Webots instance
ARG WEBOTS_CONTROLLER_URL
ENV WEBOTS_CONTROLLER_URL=${WEBOTS_CONTROLLER_URL}

# Source the ROS humble setup file and run the participant package
CMD . /opt/ros/humble/setup.sh && . /usr/local/webots-project/controllers/participant/install/setup.sh && ros2 launch participant robot_launch.py
