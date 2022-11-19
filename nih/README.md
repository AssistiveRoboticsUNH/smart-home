### nih
This package contains necessary launch files to run the pioneer robot and remapp the topics.

<b>Complete up</b> </br>
This will run all the necessary node and launch files
```
ros2 launch nih robot.launch.py
```



<b>Run pioneer only</b> </br>
This will run the p2os driver, publish motor state 1, publish robot_state_publisher
```
ros2 launch nih pioneer.launch.py
```

<b>Run sick laserscan only</b><br/>
This will run the sick driver and remap /sick_lms_5xx/scan to /scan
```
ros2 launch nih sick_lms_5xx.launch.py
```

