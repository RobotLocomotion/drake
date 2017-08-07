This directory contains an example simulation of the PR2. This code has been simplified from code at the fork TristanThrush/drake. That fork contains an interface between the high-level problem solver BHPN (http://lis.csail.mit.edu/pubs/tlp/IJRRBelFinal.pdf) and Drake.

The simulation loads a PR2, a small rectangular prism, and two tables. 
To build the simulation, enter $ bazel build //drake/examples/pr2:pr2_simulator
To run the simulation, enter $ ./bazel-bin/drake/examples/pr2/pr2_simulator


This directory also contains an lcm log of robot_plan_t's from one of BHPN's solution when BHPN was asked to meet the goal state of the rectangular prism resting on the second table.

BHPN's solution consists of PR2 picking the rectangular prism up from one table and placing it on another.
To run BHPN's solution, run the simulation, and then enter $ lcm-logplayer bhpn_generated_pick_and_place_plans_log.
