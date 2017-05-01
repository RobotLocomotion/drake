The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

To run the demo, simply execute
```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/
monolithic_pick_and_place_demo --box_choice=1
 --orientation=-0.0
```

Command line arguments can be passed for : `box_choice` an integer between 1
and 3 (corresponding to small, medium and large boxes as manipulation
targets),and for `orientation` - a decimal value for the yaw orientation on
the table in radians.
