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

Limits have been added to the xacro files, and then manually merged into the
hand-edited files.

### Velocity and Effort Limits

Velocity and effort limits were derived from the third table at page 30 of the
followking KUKA brochure:

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

### Acceleration Limits

Acceleration limits were derived from experimental results listed in Table 4
(page 50) of the following Master's thesis:

* "Including a Collaborative Robot in Digital Twin Manufacturing Systems",
Christian Larsen, Gothenburg, Sweden 2019. (URL:
<https://odr.chalmers.se/bitstream/20.500.12380/256658/1/256658.pdf>, Accessed
on 2022-01-15)

The average acceleration limits are used, and rounded to second decimal
place for radians.

For the record, the rounded average acceleration limits:

|Axis data  | deg/s^2 | rad/s^2 |
|-----------|--------:|--------:|
|Axis 1 (A1)|490.77   |8.57     |
|Axis 2 (A2)|490.80   |8.57     |
|Axis 3 (A3)|500.77   |8.74     |
|Axis 4 (A4)|650.71   |11.36    |
|Axis 5 (A5)|700.73   |12.23    |
|Axis 6 (A6)|900.66   |15.72    |
|Axis 7 (A7)|900.69   |15.72    |

### Rotor Inertia and Gear Ratio
The rotor inertias and gear ratios are estimated based on the specifications of a similar robot, i.e. DLR LWR III (link
below). The motors and gears are assumed to be the RoboDrive ILM series and Harmonic Drive CSG series. These values are
being validated experimentally.
* "DLR LWR III. (URL:
<https://www.dlr.de/rm/en/desktopdefault.aspx/tabid-12464/21732_read-49777/>, Accessed on 2023-05-08)

The motor and gear types:
|Axis data  | motor      | gear            | gear ratio | rotor inertia (kg m^2)|
|-----------|-----------:|----------------:|-----------:|----------------------:|
|Axis 1 (A1)|ILM 85x23   |CSG-32-160-2A-GR |160         |1.321e-4               |
|Axis 2 (A2)|ILM 85x23   |CSG-32-160-2A-GR |160         |1.321e-4               |
|Axis 3 (A3)|ILM 70x18   |CSG-32-160-2A-GR |160         |1.321e-4               |
|Axis 4 (A4)|ILM 70x18   |CSG-32-160-2A-GR |160         |1.321e-4               |
|Axis 5 (A5)|ILM 70x18   |CSG-32-100-2A-GR |100         |1.321e-4               |
|Axis 6 (A6)|ILM 50x08   |CSG-20-160-2A-GR |160         |4.54e-5                |
|Axis 7 (A7)|ILM 50x08   |CSG-20-160-2A-GR |160         |4.54e-5                |

