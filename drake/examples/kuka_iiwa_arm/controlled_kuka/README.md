# Controlled KUKA iiwa Arm Demo

## About This Demo

This demo shows the KUKA iiwa arm following a simple IK trajectory. It is
controlled using a PID joint position controller with gravity compensation.

## How To Run The Demo

Open two terminals. In the first terminal, start Drake Visualizer:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer

In the second terminal, start the simulation:

    $ cd drake-distro
    $ ./build/drake/bin/kuka_demo
