
This directory contains an example program that illustrates how to use Drake's
YAML configuration fragments to set up and run a simulator.

To see the demo, run:

$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //examples/hardware_sim:demo

At the moment, the capabilities demonstrated by the example are somewhat
limited. We will be adding more features in the near future.

Note that neither the hardware_sim program nor its scenario schema are
installed as part of Drake releases. We expect and encourage users to
copy these files into their own projects and customize the code for
their specific needs.

We also have a Python flavor of this example under development. It is
not yet feature-complete.
