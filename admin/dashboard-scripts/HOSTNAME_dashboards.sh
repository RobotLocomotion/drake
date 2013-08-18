#!/bin/bash

cd $(dirname $0)

./run_ctest_with_drc_environment.sh drc-trunk-Continuous/software/config/drc_environment.sh paladin02_trunk_continuous.cmake
