
This directory contains example programs that illustrate how to use Drake's
YAML configuration fragments to set up and run a simulator.

The same demo is available independently as either C++ or Python.

To see the C++ demo, run:

$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //examples/hardware_sim:demo_cc

To see the Python demo, run:

$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //examples/hardware_sim:demo_py

At the moment, the capabilities demonstrated by the example are somewhat
limited. We will be adding more features in the near future.

Note that neither the hardware_sim program nor its scenario schema are
installed as part of Drake releases. We expect and encourage users to
copy these files into their own projects and customize the code for
their specific needs.

TODO(jwnimmer-tri) Add demonstrations of using the extra scenario text.
