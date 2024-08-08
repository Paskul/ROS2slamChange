# Use ROS 2 Humble base image
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
        wget \
        iproute2 \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-rviz2 \
        ros-humble-turtlebot3 \
        ros-humble-turtlebot3-msgs \
        ros-humble-turtlebot3-simulations \
        ros-humble-gazebo-ros-pkgs \
        && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install setuptools==58.2.0 numpy==1.21.5 control

RUN mkdir /ros_workspace

COPY . /ros_workspace/src/

# Set the workspace as the working directory
WORKDIR /ros_workspace

# Build your ROS workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; rosdep install -i --from-path src --rosdistro humble -y; colcon build --symlink-install'

# Source the ROS 2 and workspace setup scripts directly in the ENTRYPOINT
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_workspace/install/setup.bash && exec \"$@\"", "bash"]

# Use CMD to specify a default action
# CMD ["bash"]
