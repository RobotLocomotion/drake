# Simple Bazel-Jupyter Integration

This shows simple Bazel-Jupyter integration. Goals:

*   Permit interactive notebook sessions using Bazel dependencies, being able
to save the notebook in the source tree, using `bazel run` / `./bazel-bin`.
*   Permit testing the notebook via `bazel test`.

## Creating a Notebook

Create a notebook as you would normally; consider copying
`./tools/jupyter/example.ipynb` to the package of your choice. Next, you will
need to add the notebook to your package's `BUILD.bazel` file, which is
illustrated below.

## Adding Bazel Targets

The file `jupyter_py.bzl` contains the Skylark macro `drake_jupyter_py_binary`
which defines a Python target to run the notebook. As is always the case,
ensure that you add the appropriate Python dependencies (`deps = [...]`) and
data dependencies (`data = [...]`). All notebooks should be tested by using
`add_test_rule = 1`.

As an example:

    load("//tools/jupyter:jupyter_py.bzl", "drake_jupyter_py_binary")

    drake_jupyter_py_binary(
        name = "example",
        add_test_rule = 1,
        deps = [":example_library"],
    )

## Running Notebooks

To run a notebook interactively, being able to save the notebook and access
dependencies, use `bazel run` or `./bazel-bin`. As an example:

    # Bazel Run
    bazel run //tools/jupyter:example

    # Binary
    # N.B. You must manually re-build if you've changed the Bazel targets.
    ./bazel-bin/tools/jupyter/example

If you save the notebook, it will save to the original file in the source tree.

To test a notebook (non-interactive), either use `bazel test` with the test
target (`{name}_test`) or pass `--test` to the original binary:

    # Bazel Test
    bazel test //tools/jupyter:py/example_test

    # Binary
    ./bazel-bin/tools/jupyter/example --test

Note that this will generally not output anything to the screen aside from
errors. This should be used as a way to see if your notebook has been broken.

## Committing to Git

If you commit a Jupyter notebook, please ensure that you clear all output
cells to have a focused review, minimize the notebook size (since binary
artifacts, such as plots, are stored in outputs as base64), and minimize the
diff itself (because output numbers may change, etc.).

The easiest way to do this is make a keyboard shortcut:

*   Within a notebook, select `Help > Edit Keyboard Shortcuts`.
*   Scroll to `clear all cells output`, click `+`, and type `K,K` (or whatever
shortcut you want), press Enter, and click OK.
*   When you are saving your notebook to commit, to clear output, you can press
`ESC + K + K` to clear the outputs. (This does not change the state of your
kernel, so you can still access variables that you're working with.)

[//]: # "TODO(eric.cousineau): Figure out better way to review notebooks."
