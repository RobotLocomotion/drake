# Drake Tutorials

This provides more in-depth documentation on top of the
[Drake Documentation's Tutorials](https://drake.mit.edu/#tutorials) section.

## Running the Tutorials Locally

After Drake version v0.40.0 or newer, you'll be able to run Drake's tutorials
locally.  (Prior to that, only nightly builds will offer local tutorials.)
In the meantime, please use "Running and Viewing the Notebooks Online", below.

To run the tutorials locally, install Drake via one of the
[installation methods](https://drake.mit.edu/installation.html)
(e.g., [pip install drake](https://drake.mit.edu/pip.html)),
be sure your `PYTHONPATH` has been set per the installation instructions,
and then run `python3 -m pydrake.tutorials` to launch a Jupyter browser.

## Running and Viewing the Notebooks Online

We currently support running the tutorials on a number of cloud notebook services:

[![Deepnote](https://deepnote.com/buttons/launch-in-deepnote-white-small.svg)](https://deepnote.com/project/Tutorials-K0_FCa7yQX2kDWBx3-2RmQ/%2Findex.ipynb)
[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release-binder?urlpath=/tree/tutorials/index.ipynb)
[![nbviewer](https://img.shields.io/badge/view%20on-nbviewer-brightgreen.svg)](https://nbviewer.jupyter.org/github/RobotLocomotion/drake/blob/nightly-release/tutorials/index.ipynb)

We include (online) platform-specific notes below.

### Deepnote

We are currently transitioning to making Deepnote our preferred method of
hosting for the tutorials.  It allows provisioning via Docker, which makes it
more stable/maintainable than alternatives like Google Colab.  The user
interface is also excellent for concurrent programming with your friends.

### Binder

Since Binder uses the `robotlocomotion/drake:latest` image on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake) that is published
once a day from the `nightly-release-binder` branch, it may be missing features
used by notebooks on `master`. These will be available the next day when the
`nightly-release-binder` branch is automatically updated.

Binder lets you browse notebooks, but you must wait for about 10-20s for
it to provision a kernel and let you see the notebook.

Note: Meshcat is not currently supported on Binder, since it does not expose the relevant network ports.
<!-- TODO(russt) -- this would almost certainly work with ngrok, if we want to handle Binder in the StartMeshcat logic.  But Binder support will likely be dropped soon. -->

### nbviewer

If you are looking to browse among the notebooks with minimal wait time,
nbviewer is highly recommended, as you can also launch Binder directly from
nbviewer using the
<img width="15px" height="15px"
    src="https://nbviewer.jupyter.org/static/img/icon-binder-color.png"/>
button at the top-right.

### Github notebook preview

GitHub's interface lets you render notebooks,
but there may be a significant delay, it tends to obfuscate links with its
rendering server (`render.githubusercontent.com`) such that they are unusable,
and does not provide simple anchors for headings.

## For Developers

- To rebuild from source and run tutorials using Bazel, use e.g.:
```
bazel run //tutorials:mathematical_program
```

- The first cell of each notebook should be a Markdown cell with the tutorial's title and
  the following preamble:
```
For instructions on how to run these tutorial notebooks, please see the
[README](https://github.com/RobotLocomotion/drake/blob/nightly-release/tutorials/README.md).
```

- Do not use `%matplotlib notebook`.  It is not supported in Deepnote.
https://github.com/RobotLocomotion/drake/blob/master/geometry/optimization/graph_of_convex_sets.h#L296

- For the pull request that adds the notebook(s), please include a `nbviewer`
link to the directory on your fork and branch, e.g.,
`https://nbviewer.jupyter.org/github/{user}/drake/blob/{branch}/tutorials/`

- When you add a new notebook, add a link to the notebook in `index.ipynb`.
  
- If appropriate, add a Binder link to the notebook (in the `nightly-release-binder`
branch) in the relevant documentation in `/doc`, e.g.,
`https://mybinder.org/v2/gh/RobotLocomotion/drake/nightly-release-binder?urlpath=/tree/tutorials/{notebook}.ipynb`
<!-- TODO(russt): Update this recommendation to have links go to Deepnote instead of Binder -->

- Once your PR has landed, use this [maintainers
  notebook](https://deepnote.com/project/Tutorials-K0_FCa7yQX2kDWBx3-2RmQ/%2F.for_maintainers.ipynb)
  to update Deepnote.  Drake developers should request "Edit" access through the
  Deepnote interface if they do not have it.  Note: If your updates depend on
  changes to Drake outside of the tutorials directory, then you will have to
  wait for the updated nightly binaries to update Deepnote.

### Notes on formatting

- Deepnote (and others) use KaTeX for Latex. $\begin{aligned}math here \end{aligned}$ works.
`gathered` also works; the more typical `gather` and `align` do not.

