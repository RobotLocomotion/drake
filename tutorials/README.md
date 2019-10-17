# Drake Tutorials

## Running the Tutorials

To run the tutorial, you should ensure that you have Drake avaiable, [either via Bazel or via binary packages](https://drake.mit.edu/installation.html).

To run the notebooks using Bazel, please refer to the
[Bazel-Jupyter README](../tools/jupyter/README.md#running-notebooks).
For example:

```
bazel run //tutorials:mathematical_program
```

## Viewing the Notebooks Online

The notebooks in this folder can be viewed using
[nbviewer](https://nbviewer.jupyter.org). To see them on Drake `master` on
GitHub, please visit:
https://nbviewer.jupyter.org/github/RobotLocomotion/drake/tree/master/tutorials/

Presently, these notebooks are not yet set up to be runnable using online
services. For more information, please see
[this issue](https://github.com/RobotLocomotion/drake/issues/11962).

## For Developers

When you add a notebook, please make the first cell be a Markdown cell with the tutorial's title and the following preamble:

    For instructions on how to run these tutorial notebooks, please see the
    [README](./README.md) in this folder.

If appropriate, add an `nbviewer` link to the notebook on `master` in the
documentation.
