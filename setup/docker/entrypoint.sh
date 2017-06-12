#!/bin/bash
set -e -u
[[ $# -eq 0 ]] && {
  #Needed until bazel build produces a drake-visualizer
  cd /drake-distro/build/install/bin
  ./drake-visualizer &
  sleep 3
  cd /drake-distro
  bazel run //drake/examples/Acrobot:acrobot_run_passive
} || {
  eval "$@"
}
