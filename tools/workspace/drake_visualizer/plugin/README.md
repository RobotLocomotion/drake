# Visualization

Contains the following `director` scripts for `drake_visualizer`:

*   `contact` - Contact visualization
*   `frame` - Frame visualization
*   `image` - Image visualization
*   `time` - Simulation time and real time factor display

These scripts are enabled in `//tools:drake_visualizer` by default.
They can be toggled by supplying `--use_builtin_scripts=...` to
`drake_visualizer`.

Examples:

```sh
bazel build //tools:drake_visualizer

./bazel-bin/tools/drake_visualizer [--use_builtin_scripts=all]  # All scripts
./bazel-bin/tools/drake_visualizer --use_builtin_scripts=  # No scripts
./bazel-bin/tools/drake_visualizer --use_builtin_scripts=image  # Image only
```

## Backwards Compatibility

If you update this infrastructure, please briefly ensure that the change does
not break previous spellings of these scripts:

    (
        set -eux;
        compatibility_scripts=$(echo "
            multibody/rigid_body_plant/visualization/contact_viz.py
            multibody/rigid_body_plant/visualization/show_frames.py
            multibody/rigid_body_plant/visualization/show_time.py
            systems/sensors/visualization/show_images.py
            ")
        for script in ${compatibility_scripts}; do
            ./bazel-bin/tools/drake_visualizer --script ${script} \
                --use_builtin_scripts=
        done
    )
