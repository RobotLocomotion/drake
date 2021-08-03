# Allegro Hand - Joint Control Example

The following shows a simple multiprocess setup where a simulation is run in
one process, and a simple open-loop finger-gaiting controller is run in another
process. No time synchronization is performed between the controller and the
simulator, so real-time rates may affect the resultant behavior.

This should run for a quite a few finger gaits, and remain stable in the sense
that the joint-level controllers do not cause the simulation to become unstable
in a numerical sense. Eventually, the mug may come loose, which is expected for
this simple controller setup.

To run the following example:

```sh
bazel build \
    //examples/allegro_hand/joint_control/...
    //tools:drake_visualizer
```

Run each of the following lines in separate terminals:

```sh
bazel-bin/tools/drake_visualizer

bazel-bin/examples/allegro_hand/joint_control/allegro_single_object_simulation

bazel-bin/examples/allegro_hand/joint_control/run_twisting_mug
```
