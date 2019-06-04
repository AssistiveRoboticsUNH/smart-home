# Smart Home Robot
The project is about i) designing a smart home equipped with a socially assistive robot (SAR) and serval
internet of things (IoT) devices and ii) evaluating the feasibility of using such a smart home to care for
elderly people with dementia. The SAR will execute a range of autonomous behaviors to communicate
with the occupant of the smart home as well as all IoT devices to ensure health and well-being of the
elderly occupant and the safety of the home. 

# Install:
**ROS Kinetic**  
sudo apt-get install ros-kinetic-desktop-full

**Gmapping package**  
sudo apt-get install ros-kinetic-slam-gmapping

**Navigation package (move_base, amcl)**  
sudo apt-get install ros-kinetic-navigation

**Teleop package**  
sudo apt-get install ros-kinetic-teleop-twist-joy  
sudo apt-get install ros-kinetic-teleop-twist-keyboard 


**Aria package(for rosaria)**  
sudo apt install libaria-dev

# Usage:

## Simulator:
**Bring up simulator:**  
roscore && roslaunch pioneer_shr pioneer_gazebo.launch

**Mapping in gazebo:**  
roslaunch pioneer_shr pioneer_gazebo_mapping.launch

**Autonomous navigation in gazebo:**  
roslaunch pioneer_shr auto_navigation_gazebo.launch

## Real Robot:
