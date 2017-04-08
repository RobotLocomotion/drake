The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

To run the demo, start three terminals from Drake's home directory. Then
execute:

tty1:

```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/iiwa_wsg_simulation
```

tty2:
```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/kuka_plan_runner
```

tty3:
```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_state_machine
```
