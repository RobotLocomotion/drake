This is an experimental branch that uses Python3. This is not yet
a supported configuration on Drake's `master`.

As such, if you have any questions, please ask me (Eric), as other people on
slack aren't using this code.

## Setup

Follow setup instructions:

    https://drake.mit.edu/from_source.html

### Bionic

Follow installation instructions normally.

### Xenial `virtualenv`

In any terminal that you're using:

    cd drake
    source tmp/py3.virtualenv.sh

## Python3

Configure:

    ./setup/configure_environment -f --python_bin $(which python3)

Now you can use Python3.
