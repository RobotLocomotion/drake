This page contains instructions for Drake developers & contributors, related to
improving the tutorials or adding new ones.

# Local Editing

To rebuild from source and run tutorials using Bazel, use e.g.:
```
bazel run //tutorials:mathematical_program
```

# Adding a New Tutorial

- Add a link to the notebook in `tutorials/index.ipynb`.

- In the pull request overview, include a `nbviewer` link to the directory on
your fork and branch, e.g.,
`https://nbviewer.jupyter.org/github/{user}/drake/blob/{branch}/tutorials/`

# Deploying changes

(This section is a work-in-progress.)

Once your PR has landed, use this [maintainers
notebook](https://deepnote.com/project/Tutorials-K0_FCa7yQX2kDWBx3-2RmQ/%2F.for_maintainers.ipynb)
to update Deepnote. Drake developers should request "Edit" access through the
Deepnote interface if they do not have it. Note: If your updates depend on
changes to Drake outside of the tutorials directory, then you will have to
wait for the updated nightly binaries to update Deepnote.
