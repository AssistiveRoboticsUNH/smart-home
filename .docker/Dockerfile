FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# clone source
ENV ROS_WS /home/smart_home_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src
RUN git clone https://github.com/AssistiveRoboticsUNH/smart-home.git
WORKDIR smart-home
RUN git checkout ros2
RUN vcs import < external.repos.yaml
WORKDIR $ROS_WS

# get dependencies
RUN apt update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y

RUN cd ${ROS_WS} \
    && . /opt/ros/humble/setup.sh \
    && colcon build \
    && source ${ROS_WS}/install/setup.bash

# copy entry pointer script
RUN apt install net-tools -y
RUN apt install tmux -y
RUN apt-get install iputils-ping -y
RUN adduser root dialout
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
