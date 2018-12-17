This is an experimental branch that uses Python3. This is not yet
a supported configuration on Drake's `master`.

As such, if you have any questions, please ask me (Eric), as other people on
slack aren't using this code.

## Setup

First, follow the nominal setup instructions:

    https://drake.mit.edu/from_source.html

In any terminal that you're using:

    cd drake
    source tmp/py3.setup.sh

## Testing

To see what is or isn't working on your system:

    bazel test //...

If you have all solvers set up correctly, use:

    bazel test --config=everything //...

## Running

Same as normal, but using `bazel-py3`:

    bazel run {target} -- {args}

If you want all solvers, please ensure you're using `--config=everything`.

## Installing Bindings

Note that CMake support for Python3 has not yet been tested.

If you want to try installing and using the bindings, please do:

    bazel run //:install -- /path/to/install

    # In any terminal you're using:
    export PYTHONPATH=/path/to/install/lib/python3.5/site-packages:${PYTHONPATH}

**NOTE**: Check your major-minor version; if it's not 3.5, change it in the
above path.
