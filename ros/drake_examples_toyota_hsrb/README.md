# drake_examples_toyota_hsrb

## Introduction

This ROS package contains example Drake simulations using Toyota's Human Support
Robot version B (HSRb).

## Installation

Follow the Drake/ROS installation instructions
[here](http://drake.mit.edu/from_source_ros.html). This results in Drake
being installed into a ROS workspace called `drake_catkin_workspace`.

## Examples

### Demo 1 - Passive (Uncontrolled) Robot

This loads an HSRb into Drake and starts a simulation. The robot is uncontrolled
meaning the only motions observed are those due to the effects of gravity. Two
visualizers are used: (1) Drake Visualizer and (2) RViz.

First start Drake Visualizer. This is necessary until
[#3075](https://github.com/RobotLocomotion/drake/issues/3075) is resolved.

```
roscd
cd ..
./install/bin/drake-visualizer &
```

Next start the simulation:

```
roslaunch drake_examples_toyota_hsrb passive_demo.launch
```

The robot should be visible in RViz.

To run the unit test for this demo:

```
rostest drake_examples_toyota_hsrb passive_demo_test.test
```

## Debugging

### Using `gdb`

To use `gdb` with the examples, first build the workspace using mode
`Debug`:

```
cd drake_catkin_workspace
catkin config -DCMAKE_BUILD_TYPE:STRING=Debug
catkin build
```

Start ``roscore``:

```
cd drake_catkin_workspace
source devel/setup.bash
roscore
```

Load the model onto the ROS parameter server:

```
cd drake_catkin_workspace
source devel/setup.bash
roslaunch drake_examples_toyota_hsrb load_model.launch
```

Start `gdb` and tell it to load and execute the demo. The example below runs
`demo1`:

```
cd drake_catkin_workspace
source devel/setup.bash
gdb -ex run --args `rospack find drake_examples_toyota_hsrb`/../../../devel/lib/drake_examples_toyota_hsrb/passive_demo
```
