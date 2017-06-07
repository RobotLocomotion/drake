#!/bin/bash

export LD_LIBRARY_PATH="external/director/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="external/director/lib/python2.7/dist-packages:$PYTHONPATH"

exec "external/director/bin/drake-visualizer" "$@"
