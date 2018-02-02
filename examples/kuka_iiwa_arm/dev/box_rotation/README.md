Iiwa Bi-Manual Manipulation Example : Box Rotation
==================================================

The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

Prerequisites
-------------

Ensure that you have installed the drake visualizer with
```
bazel build //tools:drake_visualizer
```

Ensure that you have built the iiwa controller with
```
bazel build //examples/kuka_iiwa_arm:iiwa_controller
```

Ensure that you have built the box rotation demo with
```
bazel build //examples/kuka_iiwa_arm/dev/box_rotation/...
```

All instructions assume that you are launching from the `drake`
workspace directory.
```
cd drake
```

Basic IIWA Box Rotation Simulation
----------------------------------

Launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

### To run the demo using terminals
Start three terminals from Drake's home directory. Then
execute:

tty1:

```
$ ./bazel-bin/examples/kuka_iiwa_arm/dev/box_rotation/iiwa_box_simulation
```

tty2:
```
$ ./bazel-bin/examples/kuka_iiwa_arm/iiwa_controller --interp_type FOH --urdf examples/kuka_iiwa_arm/dev/box_rotation/models/dual_iiwa14_primitive_sphere_visual_collision.urdf 
```
Note the ```gflag``` option ```--interp_type FOH```. This specifies a first-order hold
interpolation scheme for the piece-wise polynomial joint position source. The
box rotation demo does not currently work with other interpolation types.

tty3:
```
$ ./bazel-bin/examples/kuka_iiwa_arm/dev/box_rotation/box_rotation_demo
```

### To run the demo using `procman` 
Simply run `./run_procman.sh`. This shell script assumes the existence of
the environment variables `DRAKE_WORKSPACE` and `PROCMAN_PATH`.
See `run_procman.sh` for more info.
