Drake / ROS Car Simulation README
=================================

This README provides details about and instructions on how to run Drake's car
demonstrations. It assumes you have [installed Drake within a ROS Catkin
workspace](http://drake.mit.edu/from_source_ros.html).

Demo 1: Single Car in MIT Stata Garage
======================================

To run this demo, open two terminals. In the first terminal, execute the
following command to launch Drake and RViz. The simulation runs in
Drake while RViz serves as the visualizer.

```
$ roslaunch drake_cars_examples single_car_in_stata_garage.launch
```

In the second terminal, execute the following command to launch an application
that allows you to issue driving commands to the simulated vehicle:

```
$ rosrun ackermann_drive_teleop ackermann_drive_keyop.py 1.0 0.7 /drake/ackermann_cmd
```

You can now type arrow keys in your second terminal to issue driving commands to
the simulated vehicle.

Unit Tests
----------

To run a unit tests for Demo 1:

```
$ rostest drake_cars_examples single_car_in_stata_garage_test.test
```

Tuning Tips
===========

There are several parameters that impact the stability of the vehicle and
simulation. These parameters are loaded onto the ROS parameter server and can
be changed in
`drake_cars_examples/launch/single_car_in_stata_garage.launch`. Below are
descriptions of these parameters.

Contacts are modeled using virtual springs. The stiffness and damping gains of
these springs can be set by modifying parameters "penetration_stiffness" and
"penetration_damping".

The steering and throttle of the vehicle are PD controlled. The gains of these
controllers can be tweaked using parameters "steering_kp", "steering_kd", and
"throttle_k".

The initial time step using by the simulation's "solver" is set by parameter
"initial_step_size".