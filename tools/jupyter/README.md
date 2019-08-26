# Simple Bazel-Jupyter Integration

This shows simple Bazel-Jupyter integration. Goals:

*   Permit interactive notebook sessions using Bazel dependencies, being able
to save the notebook in the source tree, using `bazel run` / `./bazel-bin`.
*   Permit testing the notebook via `bazel test`.

## Browsing Notebooks

You can browse and edit all notebooks with the version of Jupyter that's
bundled in this workspace:

    ./bazel-bin/tools/jupyter/jupyter notebook

Note that you may not be able to run anything, as it may not include all of
the target-specific dependencies.

## Creating a Notebook

If you wish to create a notebook, simply copy `./tools/jupyter/template.ipynb`
(which is completely blank) to the package of your choice. Next, you will need
to add the notebook to your package's `BUILD.bazel` file; see below for how to
do this.

## Adding Bazel Targets

The file `jupyter_py.bzl` contains the Skylark macro `jupyter_py_binary` which
defines a Python target to run the notebook. As is always the case, ensure that
you add the appropriate Python dependencies (`deps = [...]`) and data
dependencies (`data = [...]`).

As an example:

    load("//tools/jupyter:jupyter_py.bzl", "jupyter_py_binary")

    jupyter_py_binary(
        name = "simple_notebook",
        data = ["//models"],
        deps = ...,
        add_test_rule = 1,
    )

## Running Notebooks

To run a notebook interactively, being able to save the notebook and access
dependencies, use `bazel run`, `./bazel-bin`, or `./run`. As an example:

    # N.B. You must manually re-build if you've changed the Bazel targets.
    bazel run //package:simple_notebook

If you save the notebook, it will save to the original file.

To test a notebook (non-interactive), either use `bazel test` with the test
target or pass `--test` to the original binary:

    # Bazel Test
    bazel test //package:simple_notebook_test

    # Binary
    ./bazel-bin/package/simple_notebook --test

Note that this will generally not output anything to the screen aside from
errors. This should be used as a way to see if your notebook has been broken.

## Committing to Git

If you commit a Jupyter notebook, please ensure that you clear all output
cells.

The easiest way to do this is make a keyboard shortcut:

*   Within a notebook, select `Help > Edit Keyboard Shortcuts`.
*   Scroll to `clear all cells output`, click `+`, and type `K,K` (or whatever
shortcut you want), press Enter, and click OK.
*   When you are saving your notebook to commit, to clear output, you can press
`ESC + K + K` to clear the outputs. (This does not change the state of your
kernel, so you can still access variables that you're working with.)

[//]: # "TODO(eric.cousineau): Figure out better way to review notebooks."
