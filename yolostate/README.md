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
* also publish bounding box and x,y,z on separate topic.

### Parameters
```
        self.declare_parameter('view_camera', False)
        self.declare_parameter('view_depth_camera', True)
        self.declare_parameter('camera', '/smart_home/camera/color/image_raw')
        self.declare_parameter('depth_camera', '/smart_home/camera/depth/image_raw')
        self.declare_parameter('depth_camera_info', '/smart_home/camera/depth/camera_info')

        self.declare_parameter('pub_human', '/detecthuman')
        self.declare_parameter('pub_human_depth', '/detecthumandepth')
```