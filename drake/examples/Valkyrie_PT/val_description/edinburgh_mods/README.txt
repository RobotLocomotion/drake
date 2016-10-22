***Generating a Drake (and Director) compatible URDF***

To add the required frames and contact points to the NASA urdf so that Drake and director can read it:

1. Use this fork of the official repo:
mitdrc/val_description and branch called: mfallon-mods-needed-for-drake-system

This branch inserts edinburgh_mods.xacro into the xacro.

2. Compile the urdf:
cd val_description/model/robots
rosrun xacro xacro.py valkyrie_sim.xacro -o ../urdf/valkyrie_sim.urdf

3. Currently you need to manually insert the contents of contact_points_block.txt 
   (found in this folder) inside of the link tag for leftFoot and rightFoot. 
   This is used for the quasi static balance constraint.

4. move the urdf to this location
cp valkyrie_sim.urdf PATH_TO/drc/software/models/val_description/urdf/valkyrie_sim.urdf

5. If needed you can make an sdf file like this:
gzsdf print valkyrie_sim.urdf  > valkyrie_sim.sdf

6. Generating hands.
Background: the hands located in models/common_components/hand_factory
are independent urdf models and are used when seeding hands in the director ui. 
they are generated from xacro in the same manner as for whole robot:
rosrun xacro xacro.py valkyrie_hand_left.xacro -o ../urdf/valkyrie_hand_left.urdf
rosrun xacro xacro.py valkyrie_hand_right.xacro -o ../urdf/valkyrie_hand_right.urdf
Then rename the files from .dae to .obj and then copy into the above locations

===Current Issues===
ISSUE: drake can't handle this tag, so I removed its contents:
  <gazebo reference="leftAnkleRoll">
    <sensor name="leftFootSixAxis" type="force_torque">
      <pose>0.0215646 0.0 -0.051054 3.14 0.0 -1.047</pose>
      <sensor_number id="FT001SIM"/>
      <node id="/sensors/leftFootSixAxis"/>
      <api name="TurbodriverForceTorqueSensor_v1"/>
      <always_on>true</always_on>
      <update_rate>1000</update_rate>
      <force_torque>
        <frame>sensor</frame>
      </force_torque>
    </sensor>
  </gazebo>

***IHMC URDF/SDF***
- There exists several files in val_description:
  * valkyrie_[ABC]_[hw|no_hands_hw].[urdf|sdf] ... these are the real robot URDFs
  * valkyrie_sim.[urdf|sdf]
  * these are specified in ValkyrieConfigurationRoot.sdf
- In SCS simulation, the urdf isn't read, this SDF is: valkyrie_sim.sdf
- In SCS in the real work, this SDF is read: valkyrie_B_hw.sdf
- running the following produces an sdf that is identical to the one loaded by IHMC:
  cd ~/workspace/Valkyrie/ValkyrieHardwareDrivers/bin/models/val_description/urdf
  gzsdf print valkyrie_sim.urdf > valkyrie_sim.sdf


***3D File Formats***
STL - source from NASA which came to 400MB. NOT USED
DAE - source from NASA (originally made by Marco)
VTP - used by Director
OBJ - required by Director to read the vtp files.

Development process:
1 BLEND files combining STL and new meshes created in Blender
2 DAE and OBJ output from Blender 
3 VTP created via script from Pat

1. file extensions to be changed to obj?
2. drake by itself
3. Director
4. remove STL
5. changing the normal smoothing file in director

- Maurice, december 2015
