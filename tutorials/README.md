# Drake Tutorials

## Running the Tutorials Locally

To run the tutorials locally, you should ensure that you have Drake available, [either via Bazel or via binary packages](https://drake.mit.edu/installation.html).

To run the notebooks using Bazel, please refer to the
[Bazel-Jupyter README](../tools/jupyter/README.md#running-notebooks).
For example:

```
bazel run //tutorials:mathematical_program
```

## Viewing and Running the Notebooks Online

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials)

The notebooks in this folder can be viewed and run online using
[Binder](https://mybinder.org). To see them from the Drake `nightly-release`
branch on GitHub, please visit:

<https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials>

Since Binder uses the `robotlocomotion/drake:latest` image on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake) that is published
once a day from the `nightly-release` branch, it may be missing features used by
notebooks on `master`. These will be available the next day when the
`nightly-release` branch is automatically updated.

## For Developers

*The `meshcat-visualizer` is not supported by Binder since port 7000 is not
exposed, so tutorials that use `meshcat.Visualizer.jupyter_cell()` will not
display correctly on Binder.*

When you add a notebook, please make the first cell be a Markdown cell with the tutorial's title and the following preamble:

    For instructions on how to run these tutorial notebooks, please see the
    [README](https://github.com/RobotLocomotion/drake/blob/nightly-release/tutorials/README.md).

If appropriate, add a Binder link to the notebook on the `nightly-release`
branch in the relevant documentation in `/doc`, e.g.,
```
https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release?filepath=tutorials/notebook.ipynb`
```
