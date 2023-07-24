# We use the eprosima/vulcanexus:iron-simulation image because it is light
# It has ROS2 and webots_ros2 installed
FROM eprosima/vulcanexus:iron-simulation-3.0.0

WORKDIR /usr/local/webots-project/controllers/participant

# Copies all the files of the controllers folder into the docker container
RUN mkdir -p /usr/local/webots-project/controllers
COPY . /usr/local/webots-project/controllers

# The eprosima/vulcanexus:iron-simulation Docker image does not have colcon installed
# We install it and build the participant package
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build

# Environment variable needed to connect to Webots instance
ARG WEBOTS_CONTROLLER_URL
ENV WEBOTS_CONTROLLER_URL=${WEBOTS_CONTROLLER_URL}
ENV USER=root

# Source the ROS iron setup file and run the participant package
CMD . /opt/ros/iron/setup.sh && . /usr/local/webots-project/controllers/participant/install/setup.sh && ros2 launch participant robot_launch.py
