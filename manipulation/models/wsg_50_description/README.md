**WARNING** All of the *.urdf models and *.obj meshes in this package are
deprecated and will be removed on 2020-11-01. The *.sdf files will not be
deprecated, and the *.obj associated with the *.sdf files will be stored in
the [models repo](https://github.com/RobotLocomotion/models/).

Execute the following commands to regenerate the URDF files using `xacro`. Note
that ROS Jade or newer must be used because the `xacro` scripts make use of more
expressive conditional statements [1].

```
source /opt/ros/kinetic/setup.bash
cd drake/manipulation/models
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
cd wsg_50_description
rosrun xacro xacro -o urdf/wsg_50_mesh_collision.urdf urdf/wsg_50_mesh_collision.urdf.xacro
```

[1] http://wiki.ros.org/xacro#Conditional_Blocks
