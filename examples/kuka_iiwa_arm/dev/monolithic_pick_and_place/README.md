A simulation demo of a pick-and-place of a box executed by the KUKA IIWA Arm
with a Schunk WSG gripper attached to its end.

The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

It is strongly recommended that you build drake along with SNOPT in order
to execute this demo. The default demo executable utilizes IPOPT instead
and successful execution is not guaranteed.

To run the demo, first you need to launch the drake visualizer and then
execute the demo itself in the following manner.

Build the ``drake_visualizer`` and the demo
-------------------------------------------

```
$ cd drake
$ bazel build //tools:drake_visualizer //examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:monolithic_pick_and_place_demo
```


Launching the ``drake_visualizer``
----------------------------------
Once built, the ``drake_visualizer`` can be launched by,

```
$ bazel-bin/tools/drake_visualizer&
```

Note that this method of launching the visualizer may not function correctly
on Mac OSX and a fix is expected shortly.

Launching the demo
------------------
The demo can be built and launched by using ``bazel run`` with the appropriate
full path/target specifier. For instance, from the
``drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place``
directory, you can execute the demo using :

```
bazel run :monolithic_pick_and_place_demo --config snopt -- --target=1
--orientation=-0.0
```

Alternately, if the ``monolithic_pick_and_place_demo`` has been built using
``bazel build``, the demo itself can then be launched by :

```
$ ./bazel-bin/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/
monolithic_pick_and_place_demo --target=1
 --orientation=-0.0
```

Command line arguments can be passed for : `target` an integer between 1
and 3 (corresponding to small, medium and large boxes as manipulation
targets),and for `orientation` - a decimal value for the yaw orientation on
the table in radians.
