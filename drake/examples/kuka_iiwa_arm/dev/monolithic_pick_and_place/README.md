The following instructions assume Drake was
[built using bazel](http://drake.mit.edu/bazel.html?highlight=bazel).

It is strongly recommended that you build drake along with SNOPT in order
to execute this demo. The default demo executable utilizes IPOPT instead
and the successful execution is not guaranteed.

To run the demo, first you need to launch the drake visualizer and then
execute the demo itself in the following manner.

Prepare and launch the ``drake-visualizer``
------------------------------------------

The ``drake-visualizer`` is only available via a CMake build.  We recommend
that you run a CMake build per the Drake instructions, using the same source
tree as this demo.  In that case, the ``drake-visualizer`` will automatically
be discovered by the bazel build system and can be launched by :

```
$ cd drake-distro
$ bazel-bin/external/drake_visualizer/drake-visualizer &
```

If the build system is unable to find the ``drake-visualizer``, you will see a
message like:

```
soft_failure.bzl: @drake_visualizer//:drake-visualizer does not work because
  drake-distro/build/install/bin/drake-visualizer was missing
```

In this case you will have to manually generate some other build of the
``drake-visualizer`` and launch the app externally.

Launching the demo
------------------
The demo can be launched by :

```
$ ./bazel-bin/drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/
monolithic_pick_and_place_demo --box_choice=1
 --orientation=-0.0
```

or alternatively, navigate to the
`drake-distro/drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place_demo/`
directory and then execute :

```
bazel run :monolithic_pick_and_place_demo --config snopt -- --box_choice=1
--orientation=-0.0
```

Command line arguments can be passed for : `box_choice` an integer between 1
and 3 (corresponding to small, medium and large boxes as manipulation
targets),and for `orientation` - a decimal value for the yaw orientation on
the table in radians.
