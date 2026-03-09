#!/bin/sh

# A wrapper intended for Drake Developers via `bazel run` that restores the
# current working directory before running `ruff`.

set -eu

# Find a stable path to ruff.
RUFF=$(realpath ../ruff_prebuilt+/ruff)

# Change back to the user's CWD.
cd ${BUILD_WORKING_DIRECTORY}

# Now relative paths on the command line will work correctly.
exec $RUFF "$@"
