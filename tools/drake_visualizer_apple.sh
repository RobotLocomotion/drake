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

export DYLD_LIBRARY_PATH="external/director/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
# TODO(jamiesnape): Do not hard code absolute path to vtk@8.0.
export PYTHONPATH="external/director/lib/python2.7/dist-packages:/usr/local/opt/vtk@8.0/lib/python2.7/site-packages${PYTHONPATH:+:$PYTHONPATH}"

exec "external/director/bin/drake-visualizer" "$@"
