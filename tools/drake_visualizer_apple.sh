#!/bin/bash

export DYLD_LIBRARY_PATH="external/director/lib:$DYLD_LIBRARY_PATH"
export PYTHONPATH="external/director/lib/python2.7/dist-packages:/usr/local/opt/vtk@8.0/lib/python2.7/site-packages:$PYTHONPATH"

exec "external/director/bin/drake-visualizer" "$@"
