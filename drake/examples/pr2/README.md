**This directory contains:**
  
  
* A Drake-compatible description of the PR2. This PR2 description differs from 
a standard PR2 description because it uses two prismatic joints and one 
revolute joint to move along the ground plane (named x, y, and theta, 
respectively), instead of wheels. The description also contains fully actuated 
finger joints, instead of a single prismatic joint that closes and opens finger 
joints that mimic each other.

* A description of a table.

* A description of a soda box.

* An uncontrolled simulation that simply loads the PR2 description, with 
contact parameters and integrator parameters that are reasonably stable 
for gripping objects.

* A controlled simulation that loads the PR2 description, two of the table
descriptions, and a soda description, with contact parameters and 
integrator parameters that are reasonably stable for gripping objects. In 
this simulation, most of the PR2 contact geometry is removed for performance
purposes.

* A plan sender that tells the PR2 from the controlled simulation to stretch 
out it's right arm.
  
  
  
**To use the passive simulation:**
  
  
**1.** Open a terminal and navigate to the root of the Drake distribution. Then,
build and launch the drake_visualizer, with the following commands: 

$ bazel build //tools:drake_visualizer  
$ ./bazel-bin/tools/drake_visualizer  

**2.** Open another terminal and navigate to the root of the Drake distribution.
Then, build and launch the pr2_passive_simulation, with the following commands:

$ bazel build //drake/examples/pr2:pr2_passive_simulation  
$ ./bazel-bin/drake/examples/pr2/pr2_passive_simulation  

The seconds (relative to the simulation) that the simulation will run for can 
be specified with the argument --simulation_sec=number of seconds to simulate. 
If this argument is not specified, then the simulation will run forever.
  
  
  
**To use the controlled simulation:**
  
  
**1.** Open a terminal and navigate to the root of the Drake distribution. Then,
build and launch the drake_visualizer, with the following commands: 

$ bazel build //tools:drake_visualizer  
$ ./bazel-bin/tools/drake_visualizer  
  
  
**2.** Open another terminal and navigate to the root of the Drake distribution.
Then, build and launch the pr2_simulation, with the following commands:

$ bazel build //drake/examples/pr2:pr2_simulation  
$ ./bazel-bin/drake/examples/pr2/pr2_simulation  

The seconds (relative to the simulation) that the simulation will run for can 
be specified with the argument --simulation_sec=number of seconds to simulate. 
If this argument is not specified, then the simulation will run forever.
  
  
**3.** Open another terminal and navigate to the root of the Drake distribution.
Then, build and launch the pr2_arm_stretch_plan_sender, with the following 
commands:

$ bazel build //drake/examples/pr2:pr2_arm_stretch_plan_sender  
$ ./bazel-bin/drake/examples/pr2/pr2_arm_stretch_plan_sender  

The pr2_arm_stretch_plan_sender sends a plan over lcm for the PR2 to stretch 
out it's right arm. The plan assumes that the PR2 will be in the position that
it was when the simulation started, so running the plan more than once on the 
same simulation will lead to results that make the simulation unstable.
