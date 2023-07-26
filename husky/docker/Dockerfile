FROM ros:foxy
ENV ROS_DISTRO=foxy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-dev \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-test-msgs
RUN apt-get update && apt-get install -y apt-utils && apt-get install -y curl


RUN pip3 install cython
RUN sudo apt -y install ros-$ROS_DISTRO-xacro
RUN sudo apt -y install ros-$ROS_DISTRO-controller-interface
RUN sudo apt -y install ros-$ROS_DISTRO-controller-manager
RUN sudo apt -y install ros-$ROS_DISTRO-robot-localization
RUN sudo apt -y install ros-$ROS_DISTRO-interactive-marker-twist-server
RUN sudo apt -y install ros-$ROS_DISTRO-twist-mux


RUN mkdir -p /husky_ws/src
WORKDIR /husky_ws/
RUN git clone -b foxy-devel https://github.com/husky/husky.git

RUN rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
# build the ros2 workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Update ros_entrypoint.sh
RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/husky_ws\/install\/local_setup.bash"' \
  /ros_entrypoint.sh

#RUN source /ros_entrypoint.sh

