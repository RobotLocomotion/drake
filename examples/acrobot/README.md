Drake Acrobot Examples
======================

The files in this directory implement a basic Drake acrobot (a double pendulum
actuated at its "waist") and use the acrobot to demonstrate various drake
control features.

The most common task for the acrobot is to achieve and hold an upright (both
links pointing in the positive-Z direction) position.

For more information on the acrobot, see
[Chapter 3 of Underactuated Robotics](http://underactuated.mit.edu/underactuated.html?chapter=3)


Basic Acrobot Simulation and Control
------------------------------------

The basic acrobot and its simple controllers are implemented in the files:

* `acrobot_geometry.{cc,h}`
* `acrobot_plant.{cc,h}`
* `acrobot_*_named_vector.yaml`
* `spong_controller.{cc,h}`

The acrobot plant is implemented analytically rather than from a URDF or
SDFormat file.


### `//examples/acrobot:`*run_passive*

This program runs a passive acrobot with no applied torque.  It publishes
visualization data over LCM, so if you run `//tools:drake_visualizer` it will
appear there.

```
bazel run //tools:drake_visualizer &
bazel run //examples/acrobot:run_passive
```


### `//examples/acrobot:`*run_lqr*

Like *run_passive*, this runs an acrobot and publishes its visualization
information to LCM.  Unlike *run_passive*
 * It starts very near the upright position.
 * It attaches an LQR controller that can maintain the upright position.

```
bazel run //tools:drake_visualizer &
bazel run //examples/acrobot:run_lqr
```


### `//examples/acrobot:`*run_swing_up*

Like the above, but this attaches a Spong controller to the acrobot.  The
Spong controller is a hybrid controller that performs energy shaping to swing
the acrobot up to near its vertical posture, then switches to the LQR
controller to hold it there.

```
bazel run //tools:drake_visualizer &
bazel run //examples/acrobot:run_swing_up
```


Benchmarks and Demonstrations
-----------------------------

### `//examples/acrobot:`*run_lqr_w_estimator*

This is a demonstration of how to use a Kalman filter observer so that the
controller does not need direct access to the robot's state.  The acrobot's
output is limited to joint positions, and an observer system with a model of
the acrobot estimates the model state from the observed positions.  The
controller then relies only on the estimated positions.

The observer trajectory is not visible in drake_visualizer, but can be
visualized via the `call_python_client_cli` remote python interpreter.

```
bazel run //tools:drake_visualizer &
bazel run //examples/acrobot:run_lqr_w_estimator
bazel run //common/proto:call_python_client_cli
```


### `//examples/acrobot:`*run_swing_up_traj_optimization*

This demonstrates the use of trajectory optimization with acrobot: An
optimized trajectory is computed using direct collocation to limit the
required power on the joint; the acrobot is then controlled with LQR to the
trajectory.

This demonstration requires SNOPT and will not run without it.

```
bazel run //tools:drake_visualizer &
bazel run //examples/acrobot:run_swing_up_traj_optimization
```


### `//examples/acrobot:`*benchmark_autodiff*

This is a simple benchmarking program that uses the acrobot plant to benchmark
the performance of various plant operations under autodiff.  It is used by
Drake developers to detect and avoid performance regressions.


LCM-based Control Stack
-----------------------

Real-world robotic systems in Drake are often implemented with the controller
in one process and the model in a separate process, so that the same
controller process can be run against a real robot.  An example of this
pattern is provided here.

### `//examples/acrobot:`*spong_controller_w_lcm*

This runs the same control as `run_swing_up` above, but instead of connecting
to a plant it is attached to an LCM publisher

### `//examples/acrobot:`*run_plant_w_lcm*

This implements the same plant as the acrobots above, but its control is
attached to an LCM receiver.

### Putting them together

```
bazel run //tools:drake_visualizer &
bazel run //examples/acrobot:run_plant_w_lcm &
bazel run //examples/acrobot:spong_controller_w_lcm
```


Monte Carlo Parameter Search
----------------------------

A demonstration of how to use Drake stochastic schemas to do deterministic
Monte Carlo testing.

### The scenario and output schemas

A schema is a structure that uses the mechanisms in `drake::schema` to define
a yaml language.

In C++, the scenario and output schemas are defined in the `Scenario` and
`Output` structs of [spong_sim.cc](spong_sim.cc).

A simulation runner is a binary that takes a scenario and a random seed in and
outputs the scenario that was actually run (i.e. with no stochastic elements)
and the simulation output.

`//examples/acrobot:`*spong_sim_main_cc* works this way:
```
bazel build //examples/acrobot:spong_sim_main_cc
./bazel-bin/examples/acrobot/spong_sim_main_cc --scenario examples/acrobot/test/example_stochastic_scenario.yaml --output out.yaml --random_seed 12
cat out.yaml
```

### Python

`//examples/acrobot:`*spong_sim_main_py* demonstrates a similar system in
python.  However stochastic schemas are not yet supported in python so this is
only useful as a demonstration of the python schema mechanism.

### `//examples/acrobot:`*optimizer_demo*

A demonstration of running parameter search optimization.  This loads a
scenario file and optimizes scenario parameters.  For instance, given the
scenario from `test/example_stochastic_scenario.yaml`:

```
example:
  controller_params: !UniformVector
    min: [4, 40, 4, 0.9e3]
    max: [6, 60, 6, 1.1e3]
  initial_state: !UniformVector
    min: [1.1, -0.1, -0.1, -0.1]
    max: [1.3, 0.1, 0.1, 0.1]
  t_final: 30.0
  tape_period: 0.05
```

running the command

```
bazel run //examples/acrobot:optimizer_demo -- --ensemble_size=30 --num_evaluations=300
```

will produce a (much!) better set of `controller_params` in a few minutes.

### Practical considerations

This demo is just a proof of concept, not a sensible way to solve serious
parameter optimization problems.  Optimizing a more computationally expensive
problem would want a more sophisticated optimizer (the Nevergrad collection
works well), parallel or cloud-based evaluation of the metric, and more
careful curation of the ensemble.  In addition you would want separate
training and test sets.
