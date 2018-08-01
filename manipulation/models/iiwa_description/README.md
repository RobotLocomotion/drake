Execute the following commands to regenerate the URDF files using xacro. Note
that ROS Jade or newer must be used because the xacro scripts make use of more
expressive conditional statements [1].

```
source /opt/ros/kinetic/setup.bash
cd drake/manipulation/models
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
cd iiwa_description
rosrun xacro xacro -o urdf/iiwa14_primitive_collision.urdf urdf/iiwa14_primitive_collision.urdf.xacro
rosrun xacro xacro -o urdf/iiwa14_polytope_collision.urdf urdf/iiwa14_polytope_collision.urdf.xacro
rosrun xacro xacro -o urdf/dual_iiwa14_polytope_collision.urdf urdf/dual_iiwa14_polytope_collision.urdf.xacro
```

[1] http://wiki.ros.org/xacro#Conditional_Blocks
