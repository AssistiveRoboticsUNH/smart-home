# Smart Home Robot
The project is about i) designing a smart home equipped with a socially assistive robot (SAR) and serval
internet of things (IoT) devices and ii) evaluating the feasibility of using such a smart home to care for
elderly people with dementia. The SAR will execute a range of autonomous behaviors to communicate
with the occupant of the smart home as well as all IoT devices to ensure health and well-being of the
elderly occupant and the safety of the home. 

# Install:
**ROS Kinetic**  
`sudo apt-get install ros-kinetic-desktop-full`

**Gmapping package**  
`sudo apt-get install ros-kinetic-slam-gmapping`

**Navigation package (move_base, amcl)**  
`sudo apt-get install ros-kinetic-navigation`

**Teleop package**  
`sudo apt-get install ros-kinetic-teleop-twist-joy`  
`sudo apt-get install ros-kinetic-teleop-twist-keyboard` 

**face module package**  
`sudo apt-get install ros-kinetic-people-msgs`  
`sudo apt-get install ros-kinetic-jsk-rviz-plugin`   
`sudo pip2 install face_recognition`  
`sudo pip2 install opencv-python`  

**primesense camera drive**  
`sudo apt install libopenni2-dev`  
`sudo apt install ros-kinetic-openni2-launch`  
The primesense camera has to be connect to usb2.0 port  

**Aria package(for rosaria)**  
`sudo apt install libaria-dev`

**Pull and build SHR**
```bash
mkdir -p ~/catkin_ws
cd ~/catkin_ws
git clone git@github.com:AssistiveRoboticsUNH/smart-home.git
mv smart-home src
cd ~/catkin_ws
catkin_make 
# if you use youcompleteme, you need a compile database:  
# catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

# Other pre-configuration:
**Grant usb port read permission**  
`sudo usermod -a -G dialout $USER`  
Have to reboot after.

**Laser Scan Ethernet Config**  
On onboard labtop, set ethernet ip as 192.168.0.10   

The default set for lms500 is 192.168.0.1, and we don't have to change this. The labtop ethernet have to be in the same ip domain, so anything similar to 192.168.0.x will work. Here we use 192.168.0.10.
 This is a [reference artical in chinese](https://blog.csdn.net/zhuoyueljl/article/details/75244563) about the LMS500 laser.

**Set up for remote control**  

Add this to .bashr or .zshrc:  
* on board labtop
```bash
# need to add this to onboard labtop that runs roscore
export ROS_IP=0.0.0.0

# set to localhost for onboard labtop that runs roscore
export ROS_MASTER_URI=http://localhost:11311

# IP of onboard labtop that runs roscore
export ROS_HOSTNAME=10.21.152.74
```

* remote control pc or labtop
```bash
# IP of onboard labtop that runs roscore
export ROS_MASTER_URI=http://10.21.152.74:11311

# IP of remote pc 
export ROS_HOSTNAME=10.21.98,194
```
**Set up for audio and video resource**  
`export ROS_WORKSPACE=/path/to/your/catkin_ws`  

# Usage:

## Simulator:
**Bring up simulator:**  
`roscore && roslaunch pioneer_shr pioneer_gazebo.launch`

**Mapping in gazebo:**  
`roslaunch pioneer_shr pioneer_gazebo_mapping.launch`

**Autonomous navigation in gazebo:**  
`roslaunch pioneer_shr auto_navigation_gazebo.launch`

**Visualize trajectory**    
`roslaunch pioneer_shr trajectory_vis.launch`  
(you may want to visualize the trajectory after mannually localize the robot)  

## Real Robot mapping:
**ssh into onboard labtop**

**Bring up pioneer from onboard labtop:**  
`roscore && roslaunch pioneer_shr real_mapping.launch`

**Start teleop keyboard from onboard labtop:**  
`roslaunch pioneer_shr keyboard_ctrl.launch`

**Visualize from remote PC:**  
`roslaunch pioneer_shr remote_vis.launch`

## Real Robot auto-navigation:
**ssh into onboard labtop**

**Bring up pioneer from onboard labtop via ssh:**  
`roscore && roslaunch pioneer_shr auto_navigation_real_world.launch`

**Start teleop keyboard from onboard labtop via ssh:**  
`roslaunch pioneer_shr keyboard_ctrl.launch`

**Visualize from remote PC:**  
`roslaunch pioneer_shr remote_vis.launch`

## Auto-navigation trigger by sensor:
**Do all steps in auto-navigation**

**Bring up simple_navigation_goal ros node**  
`roslaunch pioneer_shr sensor_trigger_move2goal_real.launch` (real world)  
`roslaunch pioneer_shr sensor_trigger_move2goal_gazebo.launch` (gazebo)    

## Face detection in gazebo:
**Do Autonomous navigation in gazebo:**  

**start person sim**  
`roslaunch person_sim init_standing_person.launch`  

**keyboard control for the person**  
`roslaunch person_sim move_person_standing.launch`

**run face detection**  
`roslaunch pioneer_shr face_detection_gazebo.launch`  

## Face detection in real world:
**Do Autonomous navigation in real world:**  

**run face detection**  
`roslaunch pioneer_shr face_detection_real.launch`  

## Face recognition in real world:
**Do Autonomous navigation in real world:**  

**run camera on robot labtop**  
`roslaunch pioneer_shr camera_real.launch`  

**start face recognition on remote labtop**  
`roslaunch pioneer_shr face_recognition_real_multiple.launch`  

## Mecial Protocal in simulation:
**Do face detection and face recogniton in gazebo:**  

**run approach person service**  
`roslaunch pioneer_shr action_service_gazebo.launch`  

**run executive**  
`rosrun pioneer_shr executive`  

## Mecial Protocal in real world:
**Do face detection and face recogniton in real world:**  

**run approach person service**  
`roslaunch pioneer_shr action_service_real.launch`  

**run executive**  
`rosrun pioneer_shr executive`  






