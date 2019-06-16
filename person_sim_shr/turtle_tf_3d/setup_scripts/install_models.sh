#! /bin/bash

cp -a /home/user/catkin_ws/src/turtle_tf_3d/meshes/. /home/ubuntu/.gazebo/models/meshes/
chmod 755 /home/ubuntu/.gazebo/models/meshes/turtle_*
chmod 755 /home/ubuntu/.gazebo/models/meshes/Tortuga*


# http://stackoverflow.com/questions/15951748/pydot-and-graphviz-error-couldnt-import-dot-parser-loading-of-dot-files-will
sudo pip install pyparsing==1.5.7
sudo pip install pydot==1.0.28