
This directory contains an example program that illustrates how to use the
simulation configuration fragments throughout Drake.

To see the demo, run:

$ cd drake
$ bazel run //tools:meldis -- -w &
$ bazel run //manipulation/hardware_sim -- \
  --scenario_file $(pwd)/manipulation/hardware_sim/example_scenarios.yaml \
  --scenario_name=Example1
