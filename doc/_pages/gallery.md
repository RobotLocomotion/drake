---
title: Gallery
---

If you have an example from your own work that you would like to showcase, please edit ``doc/_pages/gallery.md`` directly and submit a pull request!

<!--
TODO(eric.cousineau): Make video preview images resize according to their
aspect ratio.
-->

# Underactuated Robotics

Drake is being used to teach [Underactuated Robotics](https://underactuated.csail.mit.edu/)
at MIT.  The course textbook has
numerous examples of modeling, controlling, and analyzing many of the canonical
problems in dynamics and control for robotics.

{% comment %}
TODO(russt): Add videos of a few relevant examples.
{% endcomment %}

# Manipulation

The robotics team at TRI is exploring learning and by combining Drake with
external high-fidelity rendering, we can generate highly realistic, both
physically and visually, machine-learning training data to further our research.

{% include video.html
  url = "https://www.youtube.com/embed/5aVDWjWd0EU"
  full_width = true
%}

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

Drake is being used to teach a new [course on manipulation at MIT](https://manipulation.csail.mit.edu/).
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

# LBM Eval Benchmark

[LBM Eval](https://github.com/ToyotaResearchInstitute/lbm_eval) is a simulation
benchmark from Toyota Research Institute containing 49 tasks that measure the
performance of Large Behavior Model policies. A version of this benchmark was
used for the simulation-based evaluation in the 2025 paper
[A Careful Examination of Large Behavior Models for Multitask Dexterous Manipulation](https://toyotaresearchinstitute.github.io/lbm1/).

{% include video.html
  url = "https://github.com/user-attachments/assets/4cbe099b-3653-4030-acf2-f52f20d05889"
  full_width = true
%}

# Trajectory Optimization

Michael Posa and the UPenn DAIR lab have an implementation of DIRCON,
as described in

  Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and
  Stabilization of Trajectories for Constrained Dynamical Systems." ICRA, 2016.

*Source Code:* [https://github.com/DAIRLab/dairlib/tree/master/systems/trajectory_optimization/dircon](https://github.com/DAIRLab/dairlib/tree/master/systems/trajectory_optimization/dircon)

# Task and Motion Planning

Caelan Garrett has examples using Drake in his STRIPStream/PDDLStream
task-and-motion-planning framework.

<img class="gallery" height="224" src="https://github.com/caelan/pddlstream/raw/d0eb256e88b8b5174fbd136a82867fd9e9cebc67/images/drake_kuka.png"/>

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

# Hydroelastic contact between non-convex meshes

Drake has an example of modeling hydroelastic contact between non-convex 
meshes.

{% include video.html
  url = "https://www.youtube.com/embed/ItE5FNoNsTg"
  full_width = true
%}

*Source Code:* [drake/examples/hydroelastic/python_nonconvex_mesh](https://github.com/RobotLocomotion/drake/tree/master/examples/hydroelastic/python_nonconvex_mesh)

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

<img class="gallery" height="224px" src="https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-LgYfwVg89cfloOSocyC%2F-LhJQm219Jka_jubcY86%2F-LhJQsQLlSIty5iywZxX%2FUntitled.gif?alt=media&token=c343b42a-5927-48c9-981b-b2074ae3da56"/>

<img class="gallery" height="224px" src="https://blobscdn.gitbook.com/v0/b/gitbook-28427.appspot.com/o/assets%2F-LgYfwVg89cfloOSocyC%2F-Lhq9MPiXqZNYqPUQ9bG%2F-Lhq9Tp8S5LlQUyPQaW-%2Fcart_pole_tracking.gif?alt=media&token=d5e653f0-810e-4008-8279-f1607cb12664"/>


## Shoe simulation

A great simulation of a robot tying shoe laces. Implemented using pydrake with
the laces represented with a piecewise linear approximation.

Authored by Michelle Tan, with help from Terry Suh, Mark Peterson, Russ Tedrake,
and the team at TRI

{% include video.html
  url = "https://www.youtube.com/embed/ImKYMKyVdZc"
  full_width = true
%}

*Interactive visualization:* [https://mntan3.github.io/](https://mntan3.github.io/)

*Source Code:* [https://github.com/RobotLocomotion/gym/tree/master/gym/envs/robot_locomotion_group/drake/shoe](https://github.com/RobotLocomotion/gym/tree/master/gym/envs/robot_locomotion_group/drake/shoe)

## AcroMonk: An underactuated brachiation robot that uses Pydrake for trajectory optimization and stabilization
AcroMonk is an underactuated brachiation robot that is able to brachiate over a horizontally laid ladder with a passive gripper on the real system. 
Taking advantage of Pydrake, we have generated optimal trajectories and further stabilized them using Time-Varying Linear Quadratic Regulator (TVLQR). 
The project is open-sourced to support education and research of brachiation with easy to implement hardware and the work is published in the journal of IEEE Robotics and Automation Letters 2023 ([IEEE](https://ieeexplore.ieee.org/document/10106397), [arXiv](https://arxiv.org/abs/2305.08373)). 


<img width="400" src="https://github.com/dfki-ric-underactuated-lab/acromonk/blob/main/hardware/images/bf.gif?raw=true">


![tvlqr](https://github.com/dfki-ric-underactuated-lab/acromonk/blob/main/hardware/images/5x-tvlqr.gif?raw=true)



*Source Code:* [AcroMonk](https://github.com/dfki-ric-underactuated-lab/acromonk)

## brom_drake: A Helper Library for Pydrake (Python)

`brom_drake` is a pip-installable library that simplifies the process of debugging algorithms in `pydrake`. Some of its notable features are:
- The `DiagramWatcher` object which will automatically add loggers to your Drake diagram, simplifying the process of monitoring "what's going on" between systems. (By default, the data from each of these loggers will get saved locally to your machine after the simulation is run.)
- The `drakeify_my_urdf` function which will use open-source tools to convert many Drake-incompatible `.urdf` files into ones that are compatible with Drake.
- `Production` objects which are partially complete scenes that you can use to debug common algorithms (for example, motion planning algorithms).

<img class="gallery" height="224px" src="https://raw.githubusercontent.com/kwesiRutledge/brom_drake-py/refs/heads/main/promo/productions/motion_planning/kinematic/Chem-Lab-Demo.gif"/>

*Source Code:* [https://github.com/kwesiRutledge/brom_drake-py](https://github.com/kwesiRutledge/brom_drake-py)

## Constrained Bimanual Planning

This example demonstrates planning motions for a bimanual iiwa setup, such that the relative pose of the two end-effectors remains fixed.
It uses analytic inverse kinematics, together with Drake's trajectory optimization algorithms, to efficiently produce collision-free, kinematically valid trajectories.

<img class="gallery" src="https://raw.githubusercontent.com/cohnt/constrained-bimanual-planning-example/main/other/plan.gif"/>

Thomas Cohn, Seiji Shaw, Max Simchowitz, Russ Tedrake. "Constrained Bimanual Planning with Analytic Inverse Kinematics." ICRA, 2024.
Peter Werner, Thomas Cohn, Rebecca Jiang Tim Seyde, Max Simchowitz, Russ Tedrake, Daniela Rus. "Faster Algorithms for Growing Collision-Free Convex Polytopes in Robot Configuration Space." ISRR, 2024.

*Source Code:* [https://github.com/cohnt/constrained-bimanual-planning-example](https://github.com/cohnt/constrained-bimanual-planning-example)
