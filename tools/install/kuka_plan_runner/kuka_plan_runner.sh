#!/bin/sh

# This file is installed to <prefix>/bin.
readonly prefix=$(python -c 'import os; print(os.path.realpath(os.path.join("'"$0"'", os.pardir, os.pardir)))')

export DRAKE_RESOURCE_ROOT=${prefix}/share/drake
exec ${prefix}/bin/kuka_plan_runner "$@"
