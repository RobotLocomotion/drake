# Drake Tutorials

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RobotLocomotion/drake/master?filepath=tutorials)

## Running the Tutorials

To run the tutorials, you should ensure that you have Drake available, [either via Bazel or via binary packages](https://drake.mit.edu/installation.html).

To run the notebooks using Bazel, please refer to the
[Bazel-Jupyter README](../tools/jupyter/README.md#running-notebooks).
For example:

```
bazel run //tutorials:mathematical_program
```

## Viewing and Running the Notebooks Online

The notebooks in this folder can be viewed and run online using
[Binder](https://mybinder.org). To see them from Drake `master` on
GitHub, please visit:

<https://mybinder.org/v2/gh/RobotLocomotion/drake/master?filepath=tutorials>

## For Developers

When you add a notebook, please make the first cell be a Markdown cell with the tutorial's title and the following preamble:

    For instructions on how to run these tutorial notebooks, please see the
    [README](https://github.com/RobotLocomotion/drake/blob/master/tutorials/README.md).

If appropriate, add a Binder link to the notebook on `master` in the
documentation, e.g.,
```
https://mybinder.org/v2/gh/RobotLocomotion/drake/master?filepath=tutorials/notebook.ipynb`
```
