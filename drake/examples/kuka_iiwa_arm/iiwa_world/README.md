# IIWA World Demos

## About These Demos

These demos are of a simulation of the KUKA iiwa arm in a world populated
with various objects such as tables, simple cylinders, cuboids etc.

For now, the demo presented is:

* iiwa_world_demo : An uncontrolled KUKA iiwa arm mounted on a table
collapses on some objects placed in its vicinity, scattering them.

## How To Run The Demos

Open the terminal. First launch the Drake Visualizer and then start the demo
of choice:

    $ cd drake-distro
    $ ./build/install/bin/drake-visualizer &
    $ ./build/drake/bin/[demo name]
