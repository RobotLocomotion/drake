#!/bin/bash

set -e

# If we are outside the sandbox, then change to the same relative directory as
# we would be inside the sandbox.
if ! [ -d "external/director" ]; then
    guess_runfiles=$(dirname "$0")/drake_visualizer.runfiles/drake
    if [ -d "$guess_runfiles/external/director" ]; then
        cd "$guess_runfiles"
    else
        echo "$(basename $0) error: could not find director" 1>&2
        exit 1
    fi
fi

export LD_LIBRARY_PATH="external/director/lib:external/vtk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="external/director/lib/python2.7/dist-packages:external/vtk/lib/python2.7/site-packages${PYTHONPATH:+:$PYTHONPATH}"

exec "external/director/bin/drake-visualizer" "$@"
