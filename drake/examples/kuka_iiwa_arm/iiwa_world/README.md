# IIWA World Demos

## About These Demos

These demos are of a simulation of the KUKA iiwa arm in a world populated
with various objects such as tables, simple cylinders and cuboids etc.

For now, the demo presented is :
iiwa_world_demo : An uncontrolled The KUKA iiwa that is mounted on a table
collapses on some objects placed in its vicinity, scattering them.

## How To Run The Demos

Open two terminals. In the first terminal, start Drake Visualizer:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer

In the second terminal, start the demo of choice.

    $ cd drake-distro
    $ ./build/drake/bin/[demo name]
