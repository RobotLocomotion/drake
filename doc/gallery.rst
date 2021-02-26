:orphan:

*************
Drake Gallery
*************

.. raw:: html

	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/swfobject/2.2/swfobject.js"></script>

If you have an example from your own work that you would like to showcase, please edit ``doc/gallery.rst`` directly and submit a pull request!


Underactuated Robotics
======================

Drake is being used to teach `Underactuated Robotics
<http://underactuated.csail.mit.edu>`_ at MIT.  The course textbook has
numerous examples of modeling, controlling, and analyzing many of the canonical
problems in dynamics and control for robotics.

.. TODO(russt): Add videos of a few relevant examples.


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


Manipulation class at MIT
=========================

Drake is being used to teach a new `course on manipulation at MIT
<http://manipulation.csail.mit.edu>`_.  The course software and materials give
a complete autonomous manipulation pipeline, including deep and geometric
perception, planning, and control.  Here is a highlight video of the class
"manipulation station" (the three views are rendered from the RGB-D cameras
mounted on the station):

.. raw :: html

  <iframe width="560" height="315" src="https://www.youtube.com/embed/zUS33rvbRsc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

*Source Code:* `drake/examples/manipulation_station <https://github.com/RobotLocomotion/drake/tree/master/examples/manipulation_station>`_

..
    TODO(russt): Add link to
    https://github.com/gizatt/blender_server/tree/manipulation_station_demo
    once gizatt is happy with it.

Trajectory Optimization
=======================

Michael Posa and the UPenn DAIR lab have an implementation of DIRCON,
as described in

  Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and
  Stabilization of Trajectories for Constrained Dynamical Systems." ICRA, 2016.

*Source Code:* https://github.com/DAIRLab/dairlib-public/tree/master/systems/trajectory_optimization

Task and Motion Planning
========================

Caelan Garrett has examples using Drake in his STRIPStream/PDDLStream
task-and-motion-planning framework.

.. raw :: html

  <img height="224" src="https://github.com/caelan/pddlstream/raw/d0eb256e88b8b5174fbd136a82867fd9e9cebc67/images/drake_kuka.png"/>

*Source Code:* https://github.com/caelan/pddlstream#drake


Modelling Closed Loop Topologies
================================

Drake has examples of combining its rigid body kinematic tree dynamics with
penalty forces to model a closed loop topology, such as a four bar linkage.

.. raw :: html

  <div>
  <iframe width="539" height="480" src="https://www.youtube.com/embed/X34hCwJ_iq8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
  <iframe width="539" height="480" src="https://www.youtube.com/embed/MGdETFQVqMg" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
  </div>

*Four Bar Source Code:* https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/four_bar

*Strandbeest Source Code:* https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/strandbeest

Third Party Documentation / Tutorials
=====================================

Greg's Tutorial (Python)
------------------------
Greg Izatt put together a nice tutorial demonstrating how to put together a
pretty complete simulation of kuka iiwa picking up a block with dynamics,
planning, control, and depth sensing, all through the pydrake interface.

.. raw :: html

  <iframe width="560" height="315" src="https://www.youtube.com/embed/JS5l5lrEhJw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


*Source Code:* https://github.com/gizatt/drake_periscope_tutorial

Zhaoyuan's Tutorial (Drake concepts, C++ examples)
--------------------------------------------------
Zhaoyuan Gu learned Drake from scratch as a 2019 TRI summer intern and
wrote this beginner's tutorial: https://drake.guzhaoyuan.com

.. raw :: html

  <img height=224 src="https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-LgYfwVg89cfloOSocyC%2F-LhJQm219Jka_jubcY86%2F-LhJQsQLlSIty5iywZxX%2FUntitled.gif?alt=media&token=c343b42a-5927-48c9-981b-b2074ae3da56"/>

  <img height=224 src="https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-LgYfwVg89cfloOSocyC%2F-Lhq9MPiXqZNYqPUQ9bG%2F-Lhq9Tp8S5LlQUyPQaW-%2Fcart_pole_tracking.gif?alt=media&token=d5e653f0-810e-4008-8279-f1607cb12664"/>
