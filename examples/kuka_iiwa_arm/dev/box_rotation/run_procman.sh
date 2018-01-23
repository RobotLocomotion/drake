#!/bin/bash

# This helper script runs the box rotation demo using the program 
# `bot-procman-sherriff`. We assume the following environment 
# variables are defined, e.g., by running the following commands: 
# export DRAKE_WORKSPACE=/home/<user>/<path-to-drake>
# export PROCMAN_PATH=/home/<user>/<path-to-bot-procman-sheriff>
#
# The environment variables `DRAKE_WORKSPACE` and `URDF` are used in
# the procman-sheriff configuration file `iiwa_dual_box_rot.pmd`.

# === UNCOMMENT THE URDF MODEL TO USE IN THIS SIMULATION ==
# Note: These various models can be helpful in visualization
#	and analysis of the box rotation simulation.
#
#export URDF=dual_iiwa14_primitive_cylinder_visual_collision.urdf
#export URDF=dual_iiwa14_primitive_cylinder_collision_only.urdf
#export URDF=dual_iiwa14_primitive_sphere_collision_only.urdf
export URDF=dual_iiwa14_primitive_sphere_visual_collision.urdf
#export URDF=dual_iiwa14_visual_only

# Run procman
$PROCMAN_PATH/bot-procman-sheriff -l iiwa_dual_box_rot.pmd
