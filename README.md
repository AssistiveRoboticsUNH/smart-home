#### NOTICE

if you delete the include folder of the workspace you need to create a intersection.txt in /home/hello-robot/smarthome_ws/install/shr_plan/share/shr_plan/include/shr_plan/intersection.txt 

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


**Pull and build SHR**
```bash
mkdir -p ~/smart-home/src
cd ~/smart-home/src
git clone git@github.com:AssistiveRoboticsUNH/smart-home.git
cd ~/smart-home 
colcon build --symlink-install

# if you are using youcompleteme so need a compile database:  
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

# Setup planner to start and shutdown

Step 1:
add a  cron job to clear out the intersection.txt file that include the predicates that need to be set true in the knowledge

run ''' crontab -e '''
and add 
1 0 * * * > /path_to_shr_plan/include/shr_plan/intersection.txt

also add a cron job to reboot the robot after wiping the intersection text 
because reboot requires priviledges then you need to run crontab with sudo
run '''sudo crontab -e '''
0 21 * * * /sbin/reboot

Step2: 
in The keyword.txt add all the predicates that indicate that a low level protocol is successful
for example Medicine protolc is sucessful if either predicates  (already_took_medicine) (already_reminded_medicine) are true so they should be added to the keyword.txt

Step3:

you need to set the two dictionaries protocol_type_ and keyword_protocol_ in action.hpp in the high_level_domain_Shutdown function.

this one should include all the protocols in :objects in the problem_high_level.pddl

const std::unordered_map<std::string, std::string> protocol_type_ = {
         {"protocol name ", "Type"}
    };
 for example 

 problem_high_level.pddl if looks like this 
  (:objects
     living_room kitchen home outside dining_room bedroom bathroom - Landmark
     am_meds pm_meds - MedicineProtocol
     breakfast - FoodProtocol
  )
  
  then protocol_type_ should look like 
    const std::unordered_map<std::string, std::string> protocol_type_ = {
            {"am_meds", "MedicineProtocol"},
            {"pm_meds", "MedicineProtocol"},
            {"breakfast", "FoodProtocol"}
    };

As for the keyword_protocol_ it maps the success keyword to the its protocol.
for example:

(already_took_medicine) (already_reminded_medicine) are success keywords for am_meds and pm_meds 
and (already_reminded_move) is a success for move_reminder

the keyword_protocol will look like this:

 const std::unordered_map<std::string, std::vector<std::string>> keyword_protocol_ = {
         {"already_took_medicine", {"am_meds", "pm_meds"}},
         {"already_reminded_medicine", {"am_meds", "pm_meds"}},
         {"already_reminded_move",{"move_reminder"}},
 };






