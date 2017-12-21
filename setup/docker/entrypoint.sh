#!/bin/bash
#This file is a script that defines the default run behavior of the Docker
#image. If no command line arguments are given, it will run the passive Acrobot
#demo, else it will execute privided arguments in a Bash shell.
set -e -u
[[ $# -eq 0 ]] && {
# TODO (brandon-northcutt) run the visualizer produced from Bazel
  cd /drake-distro
  bazel build //tools:drake_visualizer
  ./bazel-bin/tools/drake_visualizer&
  sleep 2
  bazel run //examples/acrobot:acrobot_run_passive
} || {
  eval "$@"
}
