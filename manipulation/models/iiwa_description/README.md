# IIWA Models

Execute the following commands to regenerate the URDF files using xacro. Note
that ROS Jade or newer must be used because the xacro scripts make use of more
expressive conditional statements [1].

```
source /opt/ros/kinetic/setup.bash

cd drake/manipulation/models

export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH

cd iiwa_description

rosrun xacro xacro -o urdf/iiwa14_primitive_collision.urdf \
urdf/iiwa14_primitive_collision.urdf.xacro

rosrun xacro xacro -o urdf/iiwa14_polytope_collision.urdf \
urdf/iiwa14_polytope_collision.urdf.xacro

rosrun xacro xacro -o urdf/dual_iiwa14_polytope_collision.urdf \
urdf/dual_iiwa14_polytope_collision.urdf.xacro
```

[1] http://wiki.ros.org/xacro#Conditional_Blocks

Beware when generating the URDF files as some of them might have been edited
manually.

## Additional Edits

### Velocity and Effort Limits

Velocity and effort limits have been added to the xacro files, and then manually
merged into the hand-edited files.

The limits were derived from the third table at page 30 of the followking KUKA
brochure:

* "KUKA Sensitive robotics_LBR iiwa. (URL:
<https://www.kuka.com/-/media/kuka-downloads/imported/9cb8e311bfd744b4b0eab25ca883f6d3/kuka_lbr_iiwa_brochure_en.pdf>
, Accessed on 2021-11-21)

For the record, the velocity and effort limits:

|Axis data  |Max. Torque|Max. Velocity|
|-----------|----------:|------------:|
|Axis 1 (A1)|320 Nm     |85 deg/s     |
|Axis 2 (A2)|320 Nm     |85 deg/s     |
|Axis 3 (A3)|176 Nm     |100 deg/s    |
|Axis 4 (A4)|176 Nm     |75 deg/s     |
|Axis 5 (A5)|110 Nm     |130 deg/s    |
|Axis 6 (A6)|40 Nm      |135 deg/s    |
|Axis 7 (A7)|40 Nm      |135 deg/s    |
