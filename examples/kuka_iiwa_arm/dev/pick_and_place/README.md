# Overview
The demo scenario consists of one or more Kuka iiwa arms surrounded by two or
more tables. The arm will move a target object from one table to the next
counter-clockwise about the base of the arm.

This demo uses LCM ([Lightweight Communications and
Marshalling](https://lcm-proj.github.io/)) for message passing between four
executables:
- Simulator (`lcm_pick_and_place_simulator`) - A physical simulation of the
  demo scenario.
- Planner (`lcm_pick_and_place_planner`) - A state-machine that outputs
  joint-space trajectories and gripper commands.
- Interpolator (`iiwa_controller`) - An interpolator that takes in joint-space
  trajectories and outputs joint-space position commands.
- Visualizer (`drake-visualizer`) - A 3D visualizer that displays the demo
  scenario as it's being simulated.

The signal lines in the block diagram below represent LCM communication between
the processes.

```
            ┌─────────┐ wsg command
┌──────────▶│         ├─────────┐      ┌───────────┐
│           │         │         │      │           │
│ ┌────────▶│ Planner │         │      │           │
│ │         │         │plan     │      │           │
│ │ ┌──────▶│         ├─────┐   │      │           │ vis info   ┌────────────┐
│ │ │       └─────────┘     │   └─────▶│           ├───────────▶│ Visualizer │
│ │ │                       │          │           │            └────────────┘
│ │ │    ┌──────────────────┘          │           │ optitrack
│ │ │    │                             │ Simulator ├────────────────┐
│ │ │    │  ┌──────────────┐    iiwa   │           │                │
│ │ │    └─▶│              │ command   │           │ wsg_status     │
│ │ │       │ Interpolator ├──────────▶│           ├──────────────┐ │
│ │ │    ┌─▶│              │           │           │              │ │
│ │ │    │  └──────────────┘           │           │ iiwa_status  │ │
│ │ │    │                             │           ├────────────┐ │ │
│ │ │    └──────────────┐              └───────────┘            │ │ │
│ │ │                   │                                       │ │ │
│ │ │                   │                                       │ │ │
│ │ └───────────────────┴───────────────────────────────────────┘ │ │
│ └───────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────┘
```

# Running the demo
The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).
Furthermore, it assumes that you've installed Drake by executing
```
$ bazel run //:install <drake_install_dir>
```
where `<drake_install_dir>` is the destination directory for the installation
and where `<drake_install_dir>/bin` is on your `PATH`. Installing Drake in this
manner gives us easy access to the following tools:
- `bot-procman-sheriff` a process manager
- `drake-visualizer` a 3D visualizer
- `drake-lcm-spy` a tool for inspecting the traffic over an LCM network.

To run the demo, launch a terminal in the Drake root directory and execute:
```
$ bot-procman-sheriff -l examples/kuka_iiwa_arm/dev/pick_and_place/multi-process-simulation.pmd
```
This will bring up a `bot-procman-sheriff` window from which you can control
the processes involved in the demo. You can now launch these processes in
various configurations:
- Select "Scripts" > "0.simulate-single-iiwa-scenario" from the menu bar to
  launch a scenario with one arm and four tables.
- Select "Scripts" > "1.simulate-dual-iiwa-scenario" from the menu bar to launch
  a scenario with two arms and eight tables.
- Select "Scripts" > "2.stop-simulation" from the menu bar to stop any running
  simulations, but leave the visualizer open.  You can also start and stop
  individual processes or groups of processes through the right-click menu of
  the corresponding line in the GUI.

# Modifying scenarios
Double-clicking on one of the process lines in the GUI will bring up a dialog
that allows you to inspect and/or modify the command that is executed for that
process. If you look at the "0.simulators" and "1.planners" groups, you will
find that the `lcm_pick_and_place_simulator` and `lcm_pick_and_place_planner`
commands both take a `--configuration_file` flag. For example, the
configuration file for the single-iiwa scenario is
`drake/examples/kuka_iiwa_arm/pick_and_place/configuration/single_iiwa.pick_and_place_configuration`.
Because this file-path begins with `drake/`, the executables will look for it
under the Drake root directory. You can also supply your own configuration file
from outside the Drake file hierarchy; in that case, you will need to provide
the absolute path to the file. To create your custom scenario, simply copy one
of the existing configuration files and make modifications as desired. The
configuration files are text-format protobuf files that use the message types
defined in `<drake
root>/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.proto`.
