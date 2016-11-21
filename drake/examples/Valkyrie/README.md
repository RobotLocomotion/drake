# Valkyrie

This experimental directory does not adhere to the development practices that
apply elsewhere in Drake. In particular, for PRs that affect only this
directory, one feature review is sufficient; platform review is not required.

If build or test targets in this directory break, and the PR author or oncall
buildcop cannot trivially resolve them, a GitHub issue will be assigned to
the Valkyrie team. If the issue is not resolved within 24 hours, the author
or buildcop may disable the offending targets.


To run the visualizer:
  $ cd drake-distro
  $ ./build/install/bin/drake-visualizer

To run the pd + feedforward controller:
  $ cd drake-distro
  $ ./build/drake/examples/Valkyrie/valkyrie_pd_ff_controller

To run the simulation:
  $ cd drake-distro
  $ ./build/drake/examples/Valkyrie/valkyrie_simulation

The visualizer needs to be started before the simulator.
The controller and simulator are not synchronized in any way, and they both
try to run as fast as possible.
For a bit more repeatability, start the controller first.

The controlled simulation eventually fails when the robot falls down, because
the simple controller does not account for sliding feet.

