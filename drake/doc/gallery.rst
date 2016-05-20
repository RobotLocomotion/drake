*************
Drake Gallery
*************

.. raw:: html

	<script type="text/javascript" src="https://ajax.googleapis.com/ajax/libs/swfobject/2.2/swfobject.js"></script>

If you have an example from your own work that you would like to showcase, please edit ``drake/doc/gallery.rst`` directly and submit a pull request!


Legged Robots
=============


The Rimless Wheel
-----------------

The simplest and perhaps the most elegant example of the principles of passive dynamic walking.

.. raw :: html

	<script type="text/javascript">
    swfobject.embedSWF("http://groups.csail.mit.edu/locomotion/drake/movies/rimlessWheel.swf", "rimlessWheel", "500", "200", "9.0.0"); <!-- ,"expressInstall.swf",{},{menu:"true"});-->
	</script>

	<div id="rimlessWheel">
		<p>Rimless Wheel video</p>
	</div>

*Source Code:* `drake/examples/RimlessWheel <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/RimlessWheel>`_


The Compass Gait
----------------

One of the simplest bipedal walking systems, it walks passively down a ramp or can be actuated by a torque at the hip or impulse at the toe to walk on more general terrain.

.. raw :: html

	<script type="text/javascript">
			swfobject.embedSWF("http://groups.csail.mit.edu/locomotion/drake/movies/compassGait.swf", "compassGait", "350", "200", "9.0.0");
	</script>

	<div id="compassGait">
		<p>Compass Gait video</p>
	</div>

*Source Code:* `drake/examples/CompassGait <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/CompassGait>`_


The Planar Monopod Hopper
-------------------------

A simple model of the famous hopping robots from MIT's Leg Laboratory.

.. raw :: html

	<script type="text/javascript">
			swfobject.embedSWF("http://groups.csail.mit.edu/locomotion/drake/movies/hopper.swf", "hopper", "500", "200", "9.0.0"); <!--,"expressInstall.swf",{},{loop:"true"});-->
	</script>

	<div id="hopper">
		<p>Planar Monopod Hopper video</p>
	</div>

*Source Code:* `drake/examples/PlanarMonopodHopper <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/PlanarMonopodHopper>`_


LittleDog Gait Optimization
---------------------------

.. raw :: html

	<iframe width="560" height="315" src="https://www.youtube.com/embed/Yvg35TGJuFw?rel=0" frameborder="0" allowfullscreen></iframe>

*Source Code:* `drake/examples/LittleDog <https://github.com/RobotLocomotion/LittleDog>`_  (set EXAMPLES_LITTLEDOG to ON in drake-distro cmake options to install it)


Strandbeest
-----------

A dynamical model of a robot inspired by Theo Jansen's `Strandbeest <http://www.strandbeest.com/>`_ walking creatures. The model has over 100 joints, but just one degree of freedom thanks to its complex mechanical linkages. It can also `walk passively downhill <https://www.youtube.com/watch?v=nsBxa_lxT7s>`_.

.. raw :: html

	<iframe width="560" height="315" src="https://www.youtube.com/embed/H6fL-8ScUnU?rel=0" frameborder="0" allowfullscreen></iframe>

*Source Code:* `drake/examples/Strandbeest <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/Strandbeest>`_


Unmanned Aerial Vehicles
========================

The Perching Glider
-------------------

.. raw :: html

	<script type="text/javascript">
			swfobject.embedSWF("http://groups.csail.mit.edu/locomotion/drake/movies/perchingGlider.swf", "perchingGlider", "500", "500", "9.0.0");
	</script>

	<div id="perchingGlider">
		<p>Perching Glider video</p>
	</div>

*Source Code:* `drake/examples/Glider <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/Glider>`_


Quadrotor Forest Trajectory Optimization
----------------------------------------

A simple demonstration of collision-free dynamic trajectory optimization.

.. raw :: html

	<iframe width="560" height="315" src="https://www.youtube.com/embed/bNm-Eu3RlCM?rel=0" frameborder="0" allowfullscreen></iframe>

*Source Code:* `drake/examples/Quadrotor/runDircolWObs.m <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/Quadrotor/runDircolWObs.m>`_


Quadrotor Online Obstacle Avoidance
----------------------------------------

A demonstration of using funnel libraries to dynamically navigate a simulated forest.

.. raw :: html

	<iframe width="560" height="315" src="https://youtu.be/lnvR_tWXzi4" frameborder="0" allowfullscreen></iframe>

*Source Code:* `drake/examples/Quadrotor/runOnlinePlanning.m <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/Quadrotor/runOnlinePlanning.m>`_


Quadrotor Simulation with Onboard Lidar
----------------------------------------

A demonstration of using Drake to simulate a lidar sensor onboard a quadrotor in C++.

.. raw :: html

	<iframe width="560" height="315" src="https://www.youtube.com/watch?v=oB73wTbvnHI" frameborder="0" allowfullscreen></iframe>

*Source Code:* `drake/examples/Quadrotor/runDynamics.cpp <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/Quadrotor/runDynamics.cpp>`_


Robot Manipulation
==================



Humanoid Robots
===============


Cars
====


A simulation of a dynamical car model based on the Toyota Prius body, simulated in the Drake toolkit.

.. raw :: html

	<iframe width="560" height="315" src="https://www.youtube.com/embed/dbtz6Gvs1Q8?rel=0" frameborder="0" allowfullscreen></iframe>

*Source Code:* `drake/examples/Cars <https://github.com/RobotLocomotion/drake/tree/master/drake/examples/Cars>`_


Canonical Underactuated Systems
===============================


Physics Engine
==============


Drake's rigid-body manipulator classes support frictional contact, aerodynamics, and a rich set of forces, sensors, and kinematic constraints.  This video just shows a few quick examples; the source code can be found in the examples and test directories of the Drake distribution.

.. raw :: html

	<iframe width="420" height="315" src="https://www.youtube.com/embed/M3m-rmPzbRk?rel=0" frameborder="0" allowfullscreen></iframe>


20 falling capsules.  It's not fast to simulate, but it works.

.. raw :: html

	<iframe width="560" height="315" src="https://www.youtube.com/embed/gsebSpj4KK8?rel=0" frameborder="0" allowfullscreen></iframe>






