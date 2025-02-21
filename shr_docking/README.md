# Docking for smart home robot

This package handles docking of smart home robot by fusing both camera and Infrared LED based docking.

## Installation

### Realsense Camera
Docking with camera needs Realsense ros2 package preinstalled with apriltag detection running.

### Infrared LED based docking

We are using micro_ros in esp32 for infrared sensor weight, bump sensor and voltage detection sensor data accumulation.
| **Topic**            | **Message Type** | **Description**                    |
|----------------------|:----------------:|------------------------------------|
| `/docking/ir_weight` | `Float32`        | Publish orientation wrt. station   |
| `/bump`              | `Float32`        | Limit switch state                 |
| `/charging_voltage`  | `Float32`        | Current voltage of the robot       |

These are the steps to setup the robot for IR docking:

1. #### Upload the [esp firmware](https://github.com/MnAkash/stretch_esp_firmware) using platformio

2. #### Setup micro-ros <br>
    Check if `<workspace>/src/smart-home/external/micro_ros_setup/` package is already downloaded. If does not exit, clone it here first:
    ```
    cd <workspace>/src/smart-home/external/
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git

    ```

    Now setup and build micro-ros into the system

    ```
    cd ~/<workspace>/src/smart-home/external
    ros2 run micro_ros_setup create_agent_ws.sh
    cd ~/<workspace>/
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.sh
    ```

3. #### Setup udev rules
    We need to first setup udev rule for external esp32 used for the system.<br>
    Run the scrip to setup udev rule:
    ```
    cd ~/<workspace>/src/smart-home/shr_docking/scripts/
    sudo ./setup_udev_esp32.sh
    ```
4. #### Fix lidar udev rule serial issue
    Lidar and esp32 has similar PID and VID. Defining lidar's serial number in its udev rule will fix the issue. Lidar's udev rule is defined in `/etc/udev/rules.d/91-hello-lrf.rules`. We have to add its serial number in the rule.

    4.1. **Find lidar serial number**
    ```
    udevadm info --query=all --name=/dev/hello-lrf
    ```
    You will find something like this 
    `ID_SERIAL_SHORT=d495a692628d05468b78a7974fc67ddf`

    4.2. **Replace the line**
    ```
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="hello-lrf" 
    ``` 
    with this line
    ```
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="d495a692628d05468b78a7974fc67ddf", MODE:="0777", SYMLINK+="hello-lrf" 
    ``` 


## Launch Requirements
Docking action servers are launched by `shr_plan/launch/action_server.launch.py` <br>
micro-ros and smartplug are launched by `shr_plan/launch/real_robot.launch.py` <br>
Lidar is run by nav2 launch file <br>
Realsense camera is launched by realsense launch file <br>

Make sure all of them are running to work with docking actions

## Send docking action
```
ros2 action send_goal /docking shr_msgs/action/DockingRequest {}
```

#### To Check standalone camera based docking
`ros2 action send_goal /docking_camera shr_msgs/action/DockingRequest {}`

#### To Check standalone IR LED based docking
`ros2 action send_goal /docking_ir shr_msgs/action/DockingRequest {}`

