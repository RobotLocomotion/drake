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

Perception
==========

Drake aims to have a wide suite of simulated sensors. As these tools
expand, we'll include video highlighting their functionality.

This video is a sneak preview of an RGB sensor model using an advanced
illumination model provided by `OSPRay <https://www.ospray.org/>`_. This video
was created by inserting a virtual RGB camera into the simulation (at an
arbitrary fixed position in the simulation's world frame) of the controlled
`Kuka arm <https://github.com/RobotLocomotion/drake/tree/master/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place>`_.
The material properties are simple to highlight the impact of the lighting model.
(October 2018)

.. raw :: html

  <iframe width="800" height="224" src="https://www.youtube.com/embed/UKxytyIJmq8" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


Task and Motion Planning
========================

Caelan Garrett has examples using Drake in his STRIPStream/PDDLStream
task-and-motion-planning framework.

.. raw :: html

  <img height="224" src="https://github.com/caelan/pddlstream/raw/d0eb256e88b8b5174fbd136a82867fd9e9cebc67/images/drake_kuka.png"/>

*Source Code:* https://github.com/caelan/pddlstream#drake


Third Party Documentation / Tutorials
=====================================

Greg Izatt put together a nice tutorial demonstrating how to put together a
pretty complete simulation of kuka iiwa picking up a block with dynamics,
planning, control, and depth sensing, all through the pydrake interface.

.. raw :: html

  <iframe width="560" height="315" src="https://www.youtube.com/embed/JS5l5lrEhJw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


*Source Code:* https://github.com/gizatt/drake_periscope_tutorial



