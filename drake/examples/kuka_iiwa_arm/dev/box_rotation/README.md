The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

To run the demo, start three terminals from Drake's home directory. Then
execute:

tty1:

```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/dev/box_rotation/iiwa_box_simulation
```

tty2:
```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/iiwa_controller --urdf drake/examples/kuka_iiwa_arm/dev/box_rotation/models/dual_iiwa14_primitive_sphere_visual_collision.urdf 
```

tty3:
```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/dev/box_rotation/box_rotation_demo
```
