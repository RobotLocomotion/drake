
.. image:: images/under_construction.png
	:width: 39%

.. image:: images/logo_w_text.jpg
	:width: 60%

******************************
Important Note (October, 2016)
******************************


Drake is currently undergoing a major renovation, with all of the core
libraries moving into C++.  The examples will move and the existing APIs will
change.  During this time, we recommend that users either engage deeply
(contributing examples/tests which provide coverage of your use cases) or wait
for a few months until the APIs have stabilized.  For a stable release, consider
checking out `this SHA <https://github.com/RobotLocomotion/drake/tree/last_sha_with_windows_support>`_.

This change is fantastically exciting -- Drake is becoming a mature and powerful
tool.  Thank you for your patience.


********
Overview
********

Drake ("dragon" in Middle English) is a C++ toolbox started by the
`Robot Locomotion Group <http://groups.csail.mit.edu/locomotion/>`_ at the MIT Computer Science and Artificial Intelligence Lab (CSAIL).  The :doc:`development team has now
grown significantly </credits>`, with core development led by the `Toyota Research Institute`_.
It is a collection of tools for analyzing the dynamics of our robots and building control systems for them, with a heavy emphasis on optimization-based design/analysis.

While there are an increasing number of simulation tools available for robotics, most of them function like a black box: commands go in, sensors come out.  Drake aims to simulate even very complex dynamics of robots (e.g. including friction, contact, aerodynamics, ...), but always with an emphasis on exposing the structure in the governing equations (sparsity, analytical gradients, polynomial structure, uncertainty quantification, ...) and making this information available for advanced planning, control, and analysis algorithms.  Drake provides interfaces to high-level languages (MATLAB, Python, ...) to enable rapid-prototyping of new algorithms, and also aims to provide solid open-source implementations for many state-of-the-art algorithms.  Finally, we hope Drake provides many compelling examples that can help people get started and provide much needed benchmarks.   We are excited to accept user contributions to improve the coverage.

Here is a quick summary of capabilities:

* Modeling Dynamical Systems
	* C++ `block-diagram <http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram_builder.html#details>`_  modeling environment with support for continuous, discrete, hybrid, event-triggered, and multi-rate systems
	* `Rigid-body kinematics and dynamics <http://drake.mit.edu/doxygen_cxx/group__rigid__body__systems.html>`_
		* Rigorously designed and tested, well-documented `multi-body library <http://drake.mit.edu/doxygen_cxx/group__multibody__concepts.html>`_
		* Load from :doc:`SDF / URDF models <models>` (+ a few custom tags)
		* `Contact/collisions modeled with compliant contact <http://drake.mit.edu/doxygen_cxx/group__drake__contacts.html>`_ ; **Coming soon**: time-stepping and hybrid models for rigid contact
		* Geometry queries (e.g. collision detection, contact queries, and sensor queries) for simple geometries and convex meshes;  **Coming soon**: non-convex meshes and multi-contact
		* `Rich library of kinematic and dynamic queries <http://drake.mit.edu/doxygen_cxx/class_rigid_body_tree.html>`_ (e.g. Centroidal dynamics, Center of Pressure, Kinematic Jacobians, ...)
	* `Sensor models <http://drake.mit.edu/doxygen_cxx/group__sensor__systems.html>`_ (lidar, RGB-D camera, imu, contact force/torque)
	* Hand-derived models for many canonical control dynamical systems
	* `Easily add your own models/components <https://github.com/RobotLocomotion/drake/blob/master/drake/examples/simple_continuous_time_system.cc>`_
	* For nearly all of the above we aim to expose sparsity in the governing equations and provide analytical gradients / symbolic analysis
	* **Coming soon**:
		* API upgrade from `RigidBodyTree <http://drake.mit.edu/doxygen_cxx/class_rigid_body_tree.html>`_ to `MultiBodyTree <http://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_tree.html>`_
		* Soft-body kinematics and dynamics
		* `System constraints <https://github.com/robotlocomotion/drake/issues/5297>`_ (Rigid-body systems already support some constraints, such as kinematic loops)
		* `Stochastic systems <https://github.com/RobotLocomotion/drake/issues/6374>`_ (Random source blocks already exist, but need random contexts and symbolic support).
		* More sophisticated rendering (towards photo-realism) and noise models for perception
* Core Libraries / Utilities
	* `MathematicalProgram <http://drake.mit.edu/doxygen_cxx/group__solvers.html>`_, a C++ interface wrapping many open-source and commercial solvers
	* `Symbolic Computation <http://drake.mit.edu/doxygen_cxx/namespacedrake_1_1symbolic.html>`_, Eigen-compatible scalar types for symbolic design and analysis
* Simulation and Analysis
	* `Simulation <http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html>`_ with a suite of `numerical integration routines <http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_integrator_base.html>`_
	* Local stability/controllability/observability analysis (via linearization / semi-definite programming)
	* **To be ported from MATLAB to C++**:
		* Find fixed points / trim conditions
		* Region of attraction analysis using sums-of-squares optimization
		* Finite-time verification (e.g. forward/backward reachability analysis)
	* **Coming soon**:
		* Uncertainty quantification
* Planning
	* Optimization-based inverse kinematics (fast `SQP-based methods <http://drake.mit.edu/doxygen_cxx/rigid__body__ik_8h.html>`_ and `Global IK <http://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_global_inverse_kinematics.html>`_)
	* `Trajectory optimization <http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_direct_trajectory_optimization.html>`_ (`kinematic <http://drake.mit.edu/doxygen_cxx/rigid__body__ik_8h.html#a6a69c1ef8426e4729ea1c7d4c11e6021>`_ and dynamic)
	* `Rich library of constraints <http://drake.mit.edu/doxygen_cxx/class_rigid_body_constraint.html>`_ used by all of the above
	* **To be ported from MATLAB to C++**
		* Contact-implicit trajectory optimization
		* `Mixed-integer-convex trajectory optimization <https://github.com/RobotLocomotion/drake/issues/6243>`_
		* Footstep/Gait planning for walking robots
		* Grasp optimization
		* Feedback motion planning
	* **Coming soon**:
		* `Sample-based motion planning <https://github.com/RobotLocomotion/drake/issues/3413>`_
* `Feedback Control Design <http://drake.mit.edu/doxygen_cxx/group__control__systems.html>`_
	* LQR design for fixed-points
	* `Inverse-dynamics controller with contact constraints <http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_inverse_dynamics_controller.html#details>`_ using Quadratic Programming (e.g. for humanoid whole-body control)
	* **To be ported from MATLAB to C++**
		* Time-varying LQR
		* Sums-of-squares-based feedback design
		* Value iteration algorithms (for low dimensional systems)
* `State Estimation <http://drake.mit.edu/doxygen_cxx/group__estimator__systems.html>`_
	* Recursive filter design (Kalman Filters, Luenberger observers)
	* **Coming soon**:
		* "Smoothing" estimators for nonlinear systems
* `System Identification <http://drake.mit.edu/doxygen_cxx/classdrake_1_1solvers_1_1_system_identification.html>`_
	* Least-squares "equation error" minimization for rigid body systems (with automatic extraction of identifiable lumped parameters)
	* Nonlinear "simulation error" minimization
* `Many examples / benchmarks / model systems <https://github.com/RobotLocomotion/drake/tree/master/drake/examples>`_
	* Acrobot, Cart-Pole, Bouncing balls, ...
	* Quadrotors
	* `Automotive/Traffic <http://drake.mit.edu/doxygen_cxx/group__automotive__systems.html>`_
	* Manipulation
	* **To be ported from MATLAB to C++**
		* Fixed-Wing UAVs
		* Walking Robots
		* Humanoids (most notably including the bulk of our codebase for Atlas from `MIT's entry in the DARPA Robotics Challenge <http://drc.mit.edu>`_)
* Other
	* `Message passing interfaces <http://drake.mit.edu/doxygen_cxx/group__message__passing.html>`_  (`LCM <https://github.com/lcm-proj/lcm>`_ and preliminary ROS support)

Most of these models/tools are described in `the companion textbook from an MIT course/MOOC <https://people.csail.mit.edu/russt/underactuated/underactuated.html?chapter=drake>`_.  We've also recently started populating the :doc:`gallery` (contributions welcome!).

We hope you find this tool useful.   Please engage us `via github issues <https://github.com/RobotLocomotion/drake/issues>`_ with comments, questions, success stories, and frustrations.  And please contribute your best bug fixes, features, and examples!


Citing Drake
============

If you would like to cite Drake in your academic publications, we suggest the following BibTeX citation::

	@misc{drake,
	 author = "Russ Tedrake and the Drake Development Team",
	 title = "Drake: A planning, control, and analysis toolbox for nonlinear dynamical systems",
	 year = 2016,
	 url = "http://drake.mit.edu"
	}


Acknowledgements
================

The Drake developers would like to acknowledge significant support from the `Toyota Research Institute`_, `DARPA <http://www.darpa.mil/>`_, the `National Science Foundation <https://nsf.gov/>`_, the `Office of Naval Research <http://www.onr.navy.mil/>`_, `Amazon.com <https://www.amazon.com/>`_, and `The MathWorks <http://www.mathworks.com/>`_.

.. _`Toyota Research Institute`: http://tri.global


Next steps
==========
.. toctree::
	 :maxdepth: 1

	 installation
	 developers
	 Doxygen (C++) <doxygen_cxx/index.html#://>
	 faq
	 issues
	 Mailing list <http://mailman.mit.edu/mailman/listinfo/drake-users>
	 credits
	 GitHub <https://github.com/RobotLocomotion/drake>


Using Drake from other Programming Languages
============================================
.. toctree::
		:maxdepth: 1

		python_bindings
		julia_bindings
		matlab_bindings


Documentation that may be useful but needs updating
===================================================
.. toctree::
		:maxdepth: 1

		gallery
		Introduction and Examples <http://underactuated.csail.mit.edu/underactuated.html?chapter=drake>
		design
		video_tutorials
