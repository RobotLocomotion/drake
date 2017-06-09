#!/bin/bash

export LD_LIBRARY_PATH="external/director/lib:external/vtk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="external/director/lib/python2.7/dist-packages:external/vtk/lib/python2.7/site-packages${PYTHONPATH:+:$PYTHONPATH}"

exec "external/director/bin/drake-visualizer" "$@"
