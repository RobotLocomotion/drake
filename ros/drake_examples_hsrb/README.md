# drake_examples_hsrb

## Introduction

This ROS package contains example applications of Drake being used with Toyota's
Human Support Robot version B (HSRb).

## Installation

Follow the Drake/ROS installation instructions
[here](http://drake.mit.edu/from_source_ros.html). This results in Drake
being installed into a ROS workspace called `drake_catkin_workspace`.

## Examples

### Demo 1 - Dynamics

This loads an HSRb robot into Drake and starts a dynamics simulation. The robot
is uncontrolled meaning the simulation simply shows the effects of gravity. It
also starts `RViz` for visualization and `roscore` for connecting the simulator
with the visualizer.

```
roslaunch drake_examples_hsrb demo1.launch

```

The robot should be visible in RViz. Because Drake is running at a low real-time
factor, the arm will fall down *extremely* slowly. The low real-time factor is
necessary because some bodies in the robot have very small inertias. Perhaps
once variable time-step integrators are supported, the simulation can be sped up
without becoming unstable.


## Debugging

### Using `gdb`

To use `gdb` with the examples, first build the workspace using mode
`RelWithDebInfo`:

```
cd drake_catkin_workspace
catkin config -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
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
roslaunch drake_examples_hsrb load_model.launch
```

Start `gdb` and tell it to load and execute the demo. The example below run
`hsr_demo_1_dynamics`:

```
cd drake_catkin_workspace
source devel/setup.bash
gdb -ex run --args `rospack find drake_examples_hsrb`/../../../devel/lib/drake_examples_hsrb/demo1
```

To run demo 2, replace the last command with:

```
gdb -ex run --args `rospack find drake_examples_hsrb`/../../../devel/lib/drake_examples_hsrb/demo2
```


Demo 2:
  - When compiled in Debug mode, runs with a real-time factor of:
       0.0039 on my Macbook Pro
