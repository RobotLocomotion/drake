This is an experimental branch that uses Python3. This is not yet
a supported configuration on Drake's `master`.

As such, if you have any questions, please ask me (Eric), as other people on
slack aren't using this code.

## Setup

Follow setup instructions, make sure you run `install_prereqs`.

    https://drake.mit.edu/from_source.html

### Bionic

Follow installation instructions normally.

### Xenial `virtualenv`

In any terminal that you're using:

    cd drake
    source tmp/py3.virtualenv.sh

Update `gen/environment.bzl` to use this Python binary:

    build --python_path=<PYTHON>
    build --action_env=DRAKE_PYTHON_BIN_PATH=<PYTHON>

## Python3

When building, running, or testing, pass `--config=python3`.
