# Controlled KUKA iiwa Arm Demos

## About These Demos

These demos are of a simulation of the KUKA iiwa arm following simple IK
trajectories. In each case, the simulated arm is controlled using a PID joint
position controller with gravity compensation.

The individual demos are :
1. controlled_kuka_demo : The KUKA iiwa follows an arbitrarily designed plan.
2. kuka_cartesian_waypoint_demo : The KUKA iiwa's end-effector follows a
rectangle in Cartesian space.

## How To Run The Demos

Open two terminals. In the first terminal, start Drake Visualizer:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer

In the second terminal, start the demo of choice.

    $ cd drake-distro
    $ ./build/drake/bin/[demo name]
