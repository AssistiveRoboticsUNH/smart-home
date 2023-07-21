 
## Simulator keyboard controls 
`shift+O` : open door

`shift+D` : move to door

`shift+B` : move to bedroom

`shift+C` : move to couch

`shift+K` : toggle move to kitchen position 1/2

`shift+P` : take medicine

`WASD` : nagivate camera

`right-click and drag` : pan camera

`mouse wheel` : zoom in 

  
## Managing source package dependencies 
A source package must be build in order to use it. It is recommended to manage these packages using a .repos file. In this file, git repositories are listed in a .yaml file format and can be downloaded using vcs. To install vcs you can run: 
``` 
sudo apt-get install python3-vcstool 
``` 
Once installed, you can run the following to download all of the dependencies. Run the follwing in the `smart-home` folder.
``` 
vcs import < external.repos.yaml 
``` 

## Managing binary package dependencies 
A binary package can be installed without the need to build it. Official ROS packages can be installed as binary with the following: 
``` 
sudo apt install ros-{ROS_DISRO}-{PACKAGE_NAME} 
``` 
For example,   
``` 
sudo apt install ros-humble-control 
``` 
However, it is best practice to add all binary packages to the package xml (see [example](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)) and then install binary packages with  
``` 
rosdep install --from-paths src --ignore-src -y 
``` 

Note, the above command must be run at the root of the ROS workspace. 

## Managing meshes
Use Blender to view and edit mesh files. When exporting .obj files, make sure that Z is "up" and Y is "forward". 
The values can be chosen from the export options.  


# Smart Home Robot
The project is about i) designing a smart home equipped with a socially assistive robot (SAR) and serval
internet of things (IoT) devices and ii) evaluating the feasibility of using such a smart home to provide care-giving service for
elderly people with dementia. The SAR will execute a range of autonomous behaviors to communicate
with the occupant of the smart home as well as all IoT devices to ensure health and well-being of the
elderly occupant and the safety of the home. 

# Project Related Publication
[1] Tianyi Gu, Momotaz Begum, Naiqian Zhang, Dongpeng Xu, Sajay Arthanat, and Dain P. LaRoche, An Adaptive Software Framework for Dementia-care Robots. Proceedings of the ICAPS Workshop on Planning and Robotics (PlanRob-20), 2020. 

[[pdf]](http://cs.unh.edu/~tg1034/publication/shr_PlanRob2020.pdf) [[video]](https://youtu.be/MjQJuN2I3Vo) [[talk]](https://youtu.be/_laXuQWBT8U) [[slides]](http://cs.unh.edu/~tg1034/slides/PlanRob-2020-shr-slides.pdf)

[2] Sajay Arthanat, Momotaz Begum, Tianyi Gu, Dain P. LaRoche, Dongpeng Xu, and Naiqian Zhang, Caregiver Perspectives on A Smart Home-based Socially Assistive Robot for Individuals with Alzheimer's Disease and Related Dementia. Disability and Rehabilitation: Assistive Technology, 2020.

[[pdf]](http://cs.unh.edu/~tg1034/publication/shr_sajay.pdf)

# Install:
**Speech module**  
Better TTS voices: 
Download the new voice from [here](https://universitysystemnh-my.sharepoint.com/:u:/g/personal/pac48_usnh_edu/ERrsvRkJHx1Fve_Uv4RBRQ0BOGGKMvEZCGmGE4-R7GwuyQ?e=9730gE) then extract and copy it into `/usr/share/festival/voices/english`

Additional voices can be found here: http://www.festvox.org/packed/festival/2.5/voices/

**ffmpeg**
ffmpeg

**face module package**  
`sudo apt-get install ros-kinetic-people-msgs`  
`sudo apt-get install ros-kinetic-jsk-rviz-plugin`   
`sudo pip2 install face_recognition`  
`sudo pip2 install opencv-python`  

**Speech module**  
`sudo apt-get install ros-kinetic-sound-play`
Better TTS voice: https://ubuntuforums.org/archive/index.php/t-751169.html
Download the new voice from [here](https://universitysystemnh-my.sharepoint.com/:u:/g/personal/pac48_usnh_edu/ERrsvRkJHx1Fve_Uv4RBRQ0BOGGKMvEZCGmGE4-R7GwuyQ?e=9730gE) then extract and copy it into `/usr/share/festival/voices/english`

**primesense camera drive**  
`sudo apt install libopenni2-dev`  
`sudo apt install ros-kinetic-openni2-launch`  
`sudo apt install ros-humble-depth-image-proc`
The primesense camera has to be connect to usb2.0 port  

**Aria package(for rosaria)**  
`sudo apt install libaria-dev`

**ROSPlan**  
`sudo apt install ros-kinetic-mongodb-store`
ROSPlan: https://github.com/KCL-Planning/ROSPlan

**Pull and build SHR**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:AssistiveRoboticsUNH/smart-home.git
cd ~/catkin_ws
catkin build 
# if you are using youcompleteme so need a compile database:  
# catkin build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
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

## Auto-navigation trigger by sensor (obselet):
**Do all steps in auto-navigation**

**Bring up simple_navigation_goal ros node**  
`roslaunch pioneer_shr sensor_trigger_move2goal_real.launch` (real world)  
`roslaunch pioneer_shr sensor_trigger_move2goal_gazebo.launch` (gazebo)    

## Face detection in gazebo:
**Do Autonomous navigation in gazebo**  

**start person sim**  
`roslaunch person_sim init_standing_person.launch`  

**keyboard control for the person**  
`roslaunch person_sim move_person_standing.launch`

**run face detection**  
`roslaunch pioneer_shr face_detection_gazebo.launch`  

## Face detection in real world:
**Do Autonomous navigation in real world**  

**run face detection**  
`roslaunch pioneer_shr face_detection_real.launch`  

## Face recognition in real world:
**Do Autonomous navigation in real world**  

**run camera on robot labtop**  
`roslaunch pioneer_shr camera_real.launch`  
(not needed if have face_detection running first)

**start face recognition on remote labtop**  
`roslaunch pioneer_shr face_recognition_real.launch`  

## Medcial Protocal in simulation:
**Do face detection and face recogniton in gazebo**  

**run approach person service**  
`roslaunch pioneer_shr action_service_gazebo.launch`  

**run executive**  
`rosrun pioneer_shr executive`  

## Medcial Protocal in real world:
**Do face detection and face recogniton in real world**  

**run approach person service**  
`roslaunch pioneer_shr action_service_real.launch`  

**run executive**  
`rosrun pioneer_shr executive`  

## Mid Night Protocal in real world:

**launch robot and run all service**  
`roslaunch pioneer_shr shr_real.launch`  

**launch face recognition on robot labtop**  
`roslaunch pioneer_shr face_recognition_real.launch`  

**run executive**  
`rosrun pioneer_shr executive p2`  

## rosplan simple demo:

**launch robot and run all service**  
`roslaunch pioneer_shr shr_real.launch`  

**launch planner**  
`roslaunch rosplan_shr shr.launchp`   

**run executive**  
`rosrun pioneer_shr executive pddl`  

## rosplan dry run on contigent-FF:

cd to catkin_ws/src/rosplan_shr  
`rosrun rosplan_planning_system Contingent-FF -o ./common/domain_shr_conditional.pddl -f ./common/problem_shr_conditional.pddl`  




