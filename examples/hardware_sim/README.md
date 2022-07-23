
This directory contains an example program that illustrates how to use Drake's
YAML configuration fragments to set up and run a simulator.

To see the demo, run:

$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //examples/hardware_sim:homecart

At the moment, the capabilities demonstread by the example are somewhat limited.
We will be adding more features in the near future.
