---
title: Gallery
---

<div class="aside">
  <script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/swfobject/2.2/swfobject.js"></script>
</div>
If you have an example from your own work that you would like to showcase, please edit ``doc/gallery.rst`` directly and submit a pull request!


# Underactuated Robotics

Drake is being used to teach [Underactuated Robotics](http://underactuated.csail.mit.edu/)
at MIT.  The course textbook has
numerous examples of modeling, controlling, and analyzing many of the canonical
problems in dynamics and control for robotics.

{% comment %}
TODO(russt): Add videos of a few relevant examples.
{% endcomment %}

# Manipulation

The Robotics team at TRI is working hard to close the gap between simulation and
reality.  For manipulation, one important piece is accurate simulation of
rigid-body contact.

{% include video.html
  url = "https://www.youtube.com/embed/X9QuMrx-psk"
  full_width = true
%}

{% include video.html
  url = "https://www.youtube.com/embed/b_HfjGCa0jU"
  full_width = true
%}

*Source Code:* [drake/examples/kuka_iiwa_arm](https://github.com/RobotLocomotion/drake/tree/master/examples/kuka_iiwa_arm)


# Manipulation class at MIT

Drake is being used to teach a new [course on manipulation at MIT](http://manipulation.csail.mit.edu/).
The course software and materials give
a complete autonomous manipulation pipeline, including deep and geometric
perception, planning, and control.  Here is a highlight video of the class
"manipulation station" (the three views are rendered from the RGB-D cameras
mounted on the station):

{% include video.html
  url = "https://www.youtube.com/embed/zUS33rvbRsc"
  full_width = true
%}

*Source Code:* [drake/examples/manipulation_station](https://github.com/RobotLocomotion/drake/tree/master/examples/manipulation_station)

{% comment %}
TODO(russt): Add link to
https://github.com/gizatt/blender_server/tree/manipulation_station_demo
once gizatt is happy with it.
{% endcomment %}

# Trajectory Optimization

Michael Posa and the UPenn DAIR lab have an implementation of DIRCON,
as described in

  Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and
  Stabilization of Trajectories for Constrained Dynamical Systems." ICRA, 2016.

*Source Code:* [https://github.com/DAIRLab/dairlib-public/tree/master/systems/trajectory_optimization](https://github.com/DAIRLab/dairlib-public/tree/master/systems/trajectory_optimization)

# Perception

Drake aims to have a wide suite of simulated sensors. As these tools
expand, we'll include video highlighting their functionality.

This video is a sneak preview of an RGB sensor model using an advanced
illumination model provided by [OSPRay](https://www.ospray.org/). This video
was created by inserting a virtual RGB camera into the simulation (at an
arbitrary fixed position in the simulation's world frame) of the controlled
[Kuka arm](https://github.com/RobotLocomotion/drake/tree/master/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place).
The material properties are simple to highlight the impact of the lighting model.
(October 2018)

(Update 2020) The OSPRay-based renderer had an impoverished API and was little
used. For maintenance reasons, it has been removed. If, in the future, users
feel they'd like to have it back, please post an issue and we can investigate
its restoration and completion.

{% include video.html
  url = "https://www.youtube.com/embed/UKxytyIJmq8"
  full_width = true
%}


# Task and Motion Planning

Caelan Garrett has examples using Drake in his STRIPStream/PDDLStream
task-and-motion-planning framework.

![Drake Kuka](/images/drake_kuka.png)

*Source Code:* [https://github.com/caelan/pddlstream#drake](https://github.com/caelan/pddlstream#drake)


# Modelling Closed Loop Topologies

Drake has examples of combining its rigid body kinematic tree dynamics with
penalty forces to model a closed loop topology, such as a four bar linkage.

{% include video.html
  url = "https://www.youtube.com/embed/X34hCwJ_iq8"
  full_width = true
%}

{% include video.html
  url = "https://www.youtube.com/embed/MGdETFQVqMg"
  full_width = true
%}

*Four Bar Source Code:* [https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/four_bar](https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/four_bar)

*Strandbeest Source Code:* [https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/strandbeest](https://github.com/RobotLocomotion/drake/tree/master/examples/multibody/strandbeest)

# Third Party Documentation / Tutorials

## Greg's Tutorial (Python)

Greg Izatt put together a nice tutorial demonstrating how to put together a
pretty complete simulation of kuka iiwa picking up a block with dynamics,
planning, control, and depth sensing, all through the pydrake interface.

{% include video.html
  url = "https://www.youtube.com/embed/JS5l5lrEhJw"
  full_width = true
%}

*Source Code:* [https://github.com/gizatt/drake_periscope_tutorial](https://github.com/gizatt/drake_periscope_tutorial)

## Zhaoyuan's Tutorial (Drake concepts, C++ examples)

Zhaoyuan Gu learned Drake from scratch as a 2019 TRI summer intern and
wrote this beginner's tutorial: [https://drake.guzhaoyuan.com](https://drake.guzhaoyuan.com/)

![](/images/Untitled.gif)

![Cart Pole Tracking](/images/cart_pole_tracking.gif)
