#!/bin/bash

# path to drake distro
export DRAKE_DISTRO=/home/rickcory/dev/drake-distro
export PROCMAN_PATH=/home/rickcory/dev/spartan/build/install/bin

# === CHOOSE THE COLLISION MODEL TYPE ==
#export URDF=dual_iiwa14_primitive_cylinder_visual_collision.urdf
#export URDF=dual_iiwa14_primitive_cylinder_collision_only.urdf
#export URDF=dual_iiwa14_primitive_sphere_collision_only.urdf
export URDF=dual_iiwa14_primitive_sphere_visual_collision.urdf
#export URDF=dual_iiwa14_visual_only

# Run procman
$PROCMAN_PATH/bot-procman-sheriff -l iiwa_dual_box_rot.pmd
