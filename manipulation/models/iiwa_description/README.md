# Generating

Execute the following commands to regenerate the URDF files using xacro. Note
that ROS Jade or newer must be used because the xacro scripts make use of more
expressive conditional statements [1].

```
source /opt/ros/melodic/setup.bash
cd drake/manipulation/models
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
cd iiwa_description
rosrun xacro xacro -o urdf/iiwa14_primitive_collision.urdf urdf/iiwa14_primitive_collision.urdf.xacro
rosrun xacro xacro -o urdf/iiwa14_polytope_collision.urdf urdf/iiwa14_polytope_collision.urdf.xacro
rosrun xacro xacro -o urdf/dual_iiwa14_polytope_collision.urdf urdf/dual_iiwa14_polytope_collision.urdf.xacro
```

[1] http://wiki.ros.org/xacro#Conditional_Blocks

*Warning*: This statement will not regenerate hand-edited models.

# Additonal Edits

*TODO*: Fill out other edits.

## Acceleration Limits

Acceleration limits are added to the xacro files, and then manually merged into
the hand-edited files.

The limits were derived from experimental results listed in Table 4 (p. 50) of
the following Master's thesis:

* "Including a Collaborative Robot in Digital Twin Manufacturing Systems",
Christian Larsen, Gothenburg, Sweden 2019. (URL:
<https://odr.chalmers.se/bitstream/20.500.12380/256658/1/256658.pdf>, Accessed
on 2021-08-13)

The average acceleration limits are used, and rounded down to first decimal
place.

For the record, the rounded average acceleration limits:

* `[490, 490, 500, 650, 700, 900, 900]` in deg/s^2
* `[8.5, 8.5, 8.7, 11.3, 12.2, 15.7, 15.7]` rad/s^2
