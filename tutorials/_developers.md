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

# Code Review

Code review of tutorials is done by running the tutorials.
What matters is the interactive experience,
not the json storage format of a notebook.
Discuss changes in Reviewable; however,
the "json code" in Reviewable is less relevant.

If the tutorial attaches images,
Reviewable will show the content of each image in one extremely long line
of hexadecimal code.
Ignore it.

Peek at the PR using git by:
```
git fetch upstream pull/99999/head && git checkout FETCH_HEAD
```

Run the tutorial using bazel as shown in Local Editing above.
