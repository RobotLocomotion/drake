:orphan:

*************
Drake Gallery
*************

.. raw:: html

	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/swfobject/2.2/swfobject.js"></script>

If you have an example from your own work that you would like to showcase, please edit ``doc/gallery.rst`` directly and submit a pull request!


Manipulation
============

The Robotics team at TRI is working hard to close the gap between simulation and
reality.  For manipulation, one important piece is accurate simulation of
rigid-body contact.

.. raw :: html

  <iframe width="800" height="224" src="https://www.youtube.com/embed/X9QuMrx-psk" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

  <p/>

  <iframe width="800" height="224" src="https://www.youtube.com/embed/b_HfjGCa0jU" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

*Source Code:* `drake/examples/kuka_iiwa_arm <https://github.com/RobotLocomotion/drake/tree/master/examples/kuka_iiwa_arm>`_


Trajectory Optimization
=======================

Michael Posa and the UPenn DAIR lab have an implementation of DIRCON,
as described in

  Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and
  Stabilization of Trajectories for Constrained Dynamical Systems." ICRA, 2016.

*Source Code:* https://github.com/DAIRLab/dairlib-public/tree/master/systems/trajectory_optimization


Third Party Documentation / Tutorials
=====================================

Greg Izatt put together a nice tutorial demonstrating how to put together a
pretty complete simulation of kuka iiwa picking up a block with dynamics,
planning, control, and depth sensing, all through the pydrake interface.

.. raw :: html

  <iframe width="560" height="315" src="https://www.youtube.com/embed/JS5l5lrEhJw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


*Source Code:* https://github.com/gizatt/drake_periscope_tutorial



