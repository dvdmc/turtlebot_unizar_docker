FROM osrf/ros:humble-desktop

RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt-get install apt-utils software-properties-common -y

RUN apt install git tmux tmuxinator nano iputils-ping net-tools -y
RUN echo "set -g mouse on" > /root/.tmux.conf

## ROS2 utils
RUN apt-get install python3-rosdep  \
                python3-pip     \
                python3-colcon-common-extensions \
                python3-colcon-mixin \
                ros-dev-tools -y

RUN apt-get install cppcheck lcov -y
RUN colcon mixin update default

# For allowing connection between machines, fastdds has some problem.
RUN apt-get install ros-humble-rmw-cyclonedds-cpp -y

RUN pip3 install colcon-lcov-result cpplint cmakelint

## Turtlebot2
RUN apt install ros-humble-kobuki-velocity-smoother ros-humble-sophus -y
RUN mkdir -p /root/turtlebot2_ws/src
WORKDIR /root/turtlebot2_ws/src
RUN git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git && \
git clone https://github.com/kobuki-base/kobuki_core.git && \
git clone https://github.com/kobuki-base/kobuki_ros.git && \
git clone https://github.com/stonier/ecl_core.git && \
git clone https://github.com/stonier/ecl_lite.git && \
git clone https://github.com/kobuki-base/cmd_vel_mux.git

WORKDIR /root/turtlebot2_ws
RUN rosdep install -i --from-path src --rosdistro humble -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --executor sequential

# NAV2
RUN apt update && apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
# BELOW CHANGES TO "sudo apt install ros-<ros2-distro>-nav2-minimal-tb*" from ROS Jazzy
RUN apt install ros-humble-turtlebot3-gazebo -y

# OptiTrack
RUN mkdir -p /root/optitrack_ws/src
WORKDIR /root/optitrack_ws/src
RUN git clone -b humble-devel https://github.com/MOCAP4ROS2-Project/mocap4r2
RUN git clone -b humble-devel https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs
RUN git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack

WORKDIR /root/optitrack_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --executor sequential

# Iron and older only with Gazebo Classic
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/turtlebot2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/optitrack_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws

RUN rm -rf log # remove log folder