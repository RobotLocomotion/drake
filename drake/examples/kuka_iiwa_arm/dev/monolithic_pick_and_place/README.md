A simulation demo of a pick-and-place of a box executed by the KUKA IIWA Arm
with a Schunk WSG gripper attached to its end.

The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

It is strongly recommended that you build drake along with SNOPT in order
to execute this demo. The default demo executable utilizes IPOPT instead
and successful execution is not guaranteed.

To run the demo, first you need to launch the drake visualizer and then
execute the demo itself in the following manner.

Build the ``drake-visualizer`` and the demo.
------------------------------------------

```
$ cd drake-distro
$ bazel build //tools:drake_visualizer && bazel build //drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:monolithic_pick_and_place_demo
```


Launching the ``drake-visualizer``
----------------------------------
Once built, the ``drake-visualizer`` can be launched by,

```
$ bazel-bin/tools/drake-visualizer&
```

Launching the demo
------------------
The demo itself can then be launched by :

```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/
monolithic_pick_and_place_demo --box_choice=1
 --orientation=-0.0
```

or alternatively, you can use ``bazel run`` using the appropriate
full path/target specifier. For instance, from the
``drake-distro/drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place``
directory, you can execute the demo using :

```
bazel run :monolithic_pick_and_place_demo --config snopt -- --box_choice=1
--orientation=-0.0
```

Command line arguments can be passed for : `box_choice` an integer between 1
and 3 (corresponding to small, medium and large boxes as manipulation
targets),and for `orientation` - a decimal value for the yaw orientation on
the table in radians.
