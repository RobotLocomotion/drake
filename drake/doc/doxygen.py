#!/bin/bash

# TODO(6996) This file is a compatibility shim; remove this as soon as CI no
# longer needs it.
echo "WARNING: drake/doc/doxygen.py is deprecated; use doc/doxygen.py" 1>&2
exec $(dirname "$0")/../../doc/doxygen.py "$@"
