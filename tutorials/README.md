# Drake Tutorials

This provides more in-depth documentation on top of the
[Drake Documentation's Tutorials](https://drake.mit.edu/#tutorials) section.

## Running the Tutorials Locally

To run the tutorials locally, you should ensure that you have Drake available, [either via Bazel or via binary packages](https://drake.mit.edu/installation.html).

To run the notebooks using Bazel, please refer to the
[Bazel-Jupyter README](../tools/jupyter/README.md#running-notebooks).
For example:

```
bazel run //tutorials:mathematical_program
```

## Running and Viewing the Notebooks Online

The notebooks in this folder can be run and quickly viewed online using
[Binder](https://mybinder.org) and [nbviewer](https://nbviewer.jupyter.org/).

To run or quickly view them from the Drake `nightly-release-binder` branch on GitHub:

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release-binder?urlpath=/tree/tutorials)
[![nbviewer](https://img.shields.io/badge/view%20on-nbviewer-brightgreen.svg)](https://nbviewer.jupyter.org/github/RobotLocomotion/drake/blob/nightly-release/tutorials/)

Since Binder uses the `robotlocomotion/drake:latest` image on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake) that is published
once a day from the `nightly-release-binder` branch, it may be missing features
used by notebooks on `master`. These will be available the next day when the
`nightly-release-binder` branch is automatically updated.

If you are looking to browse among the notebooks with minimal wait time,
nbviewer is highly recommended, as you can also launch Binder directly from
nbviewer using the
<img width="15px" height="15px"
    src="https://nbviewer.jupyter.org/static/img/icon-binder-color.png"/>
button at the top-right.
GitHub's interface lets you render notebooks,
but there may be a significant delay, it tends to obfuscate links with its
rendering server (`render.githubusercontent.com`) such that they are unusable,
and does not provide simple anchors for headings.
Binder lets you browse notebooks too, but you must wait for about 10-20s for
it to provision a kernel and let you see the notebook.

**Warning**: `meshcat-visualizer` is not supported by Binder since port 7000 is
not exposed, so tutorials that use `meshcat.Visualizer.jupyter_cell()` will not
display correctly on Binder.

## For Developers

When you add a notebook, please make the first cell be a Markdown cell with the tutorial's title and the following preamble:

    For instructions on how to run these tutorial notebooks, please see the
    [README](https://github.com/RobotLocomotion/drake/blob/nightly-release/tutorials/README.md).

For the pull request that adds the notebook(s), please include a `nbviewer`
link to the directory on your fork and branch, e.g.,
`https://nbviewer.jupyter.org/github/{user}/drake/blob/{branch}/tutorials/`

If appropriate, add a Binder link to the notebook (in the `nightly-release-binder`
branch) in the relevant documentation in `/doc`, e.g.,
`https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release-binder?urlpath=/tree/tutorials/{notebook}.ipynb`
