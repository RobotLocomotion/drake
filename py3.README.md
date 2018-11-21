This is an experimental branch that uses Python3. This is not yet
a supported configuration on Drake's `master`.

As such, if you have any questions, please ask me (Eric), as other people on
slack aren't using this code.

## Setup

In any terminal that you're using:

    cd drake
    source py3_setup.sh

Then, instead of using `bazel`, use the `bazel-py3` alias.

## Testing

To see what is or isn't working on your system:

    bazel-py3 test //...

If you have all solvers set up correctly, use:

    bazel-py3 test --config=everything //...

## Running

Same as normal, but using `bazel-py3`:

    bazel-py3 run {target} -- {args}

If you want all solvers, please ensure you're using `--config=everything`.

## Installing Bindings

Note that CMake support for Python3 has not yet been tested.

If you want to try installing and using the bindings, please do:

    bazel-py3 run //:install -- /path/to/install

    # In any terminal you're using:
    export PYTHONPATH=/path/to/install/lib/python3.5/site-packages:${PYTHONPATH}

**NOTE**: Check your major-minor version; if it's not 3.5, change it in the
above path.
