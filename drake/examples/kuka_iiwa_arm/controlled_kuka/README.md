# Controlled KUKA iiwa Arm Demos

## About These Demos

These demos simulate the KUKA iiwa arm following simple IK trajectories. In
each case, the simulated arm is controlled using a PID joint position
controller with gravity compensation.

The individual demos are:

1. controlled_kuka_demo: The KUKA iiwa follows an arbitrarily designed plan.
2. kuka_cartesian_waypoint_demo: The KUKA iiwa's end-effector follows a
rectangle in Cartesian space. The vertices of this rectangle are located on
a plane normal to the surface on which the iiwa is mounted and 30 centimeters
away in the x-direction of the robot's base reference frame.

## How To Run The Demos

Open the terminal. First launch the Drake Visualizer and then start the demo
of choice:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer &
    $ ./build/drake/bin/[demo name]
