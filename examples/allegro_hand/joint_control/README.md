# Allegro Hand - Joint Control Example

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
