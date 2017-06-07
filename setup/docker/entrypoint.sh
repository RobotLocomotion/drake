#!/bin/bash
# Copyright 2016-2017 Toyota Research Institute.  All rights reserved.
set -e
[[ $# -eq 0 ]] && {
  cd /drake/build/install/bin
  ./drake-visualizer &
  sleep 3
  cd /drake/build/drake/examples/Acrobot
  ./acrobot_run_passive
} || {
  eval "$@"
}
