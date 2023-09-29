
# Standalone Robot Simulations

This directory contains example programs that illustrate how to use Drake's YAML
configuration fragments to set up and run your own robot simulation.

The example shows how to simulate a robot where joint command messages arrive
from some external source. The simulation publishes simulated joint status
messages and simulated camera image messages.

The same demo is implemented in both Python and C++, for reference.

Note that the demo contains only static scene by default. The robot is
uncommanded and nothing is happening; all you'll see is the scene geometry along
with a display of the contact forces (green arrows). You can run another program
`robot_commander.py` to actuate the robot with a simple motion.


## Running the Python demo from a Drake binary release

The demo is not installed as part of Drake releases. We expect and encourage you
to copy these files into your own projects and customize them there.

(1) Use https://drake.mit.edu/installation.html to install Drake. The ``pip``
installation is a good option. Make sure your ``$PYTHONPATH`` is set correctly
per the instructions.

(2) Copy ``hardware_sim.py`` and ``example_scenarios.yaml`` into a temporary
directory:

* https://raw.githubusercontent.com/RobotLocomotion/drake/master/examples/hardware_sim/hardware_sim.py
* https://raw.githubusercontent.com/RobotLocomotion/drake/master/examples/hardware_sim/example_scenarios.yaml

(3) (Optional) Run the standalone visualizer:

```
python3 -m pydrake.visualization.meldis -w &
```

This command will open a new web browser window, initially showing an empty
scene. This "standalone" visualizer can remain running even as you repeatedly
start and stop the simulation.

(4) Run the simulation.

```
python3 hardware_sim.py --scenario_file=example_scenarios.yaml \
  --scenario_name=Demo --scenario_text='{ simulation_duration: 60 }'
```

(5) (Optional) Send actuation commands to the robot.

Copy ``robot_commander.py`` into the temporary directory and run the command in
another terminal:

* https://raw.githubusercontent.com/RobotLocomotion/drake/master/examples/hardware_sim/robot_commander.py

```
python3 robot_commander.py
```

If you didn't launch the visualizer in step (3), look for a log message like
``Meshcat listening for connections at http://localhost:7000`` and click on that
link to open a temporary visualizer (that will disappear when you close the
simulation).


## Running from a Drake source build

To use Python, run these commands:

```
$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //examples/hardware_sim:demo_py
```

To use C++, run these commands:

Run these commands:

```
$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //examples/hardware_sim:demo_cc
```

(Optionally) To actuate the robot, run the command in another terminal:
```
$ bazel run //examples/hardware_sim:robot_commander
```
