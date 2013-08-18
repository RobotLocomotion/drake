#!/bin/bash

cd $(dirname $0)

./run_ctest_with_drake_environment.sh drake-trunk-Continuous/drake/admin/dashboard-scripts/drake_environment.sh HOSTNAME_trunk_continuous.cmake
