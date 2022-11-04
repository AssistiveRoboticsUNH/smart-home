### Detect human using yolo


<b>Node: detecthuman </b>

* subscribe to camera parameter
* publish to /detecthuman

```
ros2 run yolostate detecthuman --ros-args -p camera:=/smart_home/camera/color/image_raw -p view_camera:=true
```

* Default parameters
```
 camera:=/smart_home/camera/color/image_raw
 view_camera:=true
```

<b>Node: detect_human_depth </b> <br/>
Subscribe to depth camera to estimate xyz position.

* detect human
* estimate x,y,z
* publish as tf

