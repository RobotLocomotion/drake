# Controlled KUKA iiwa Arm Demos

## About These Demos

These demos simulate the KUKA iiwa arm following simple IK trajectories. In
each case, the simulated arm is controlled using a PID joint position
controller with gravity compensation.

The individual demos are:

1. controlled_kuka_demo: The KUKA iiwa follows an arbitrarily designed plan.

## How To Run The Demos

Open the terminal. First launch the Drake Visualizer and then start the demo
of choice:

    $ cd drake
    $ bazel-bin/tools/drake-visualizer &
    $ bazel-bin/examples/kuka_iiwa_arm/controlled_kuka/[demo name]
