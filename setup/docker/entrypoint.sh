#!/bin/bash
set -eu

# This file is a script that defines the default run behavior of the Docker
# image. If no command line arguments are given, it will run the passive Acrobot
# demo; otherwise, it will execute the provided arguments in a Bash shell.
if [[ $# -eq 0 ]]; then
    cd /drake
    bazel build //tools:drake_visualizer
    ./bazel-bin/tools/drake_visualizer &
    sleep 2
    bazel run //examples/acrobot:acrobot_run_passive
else
    exec "$@"
fi
