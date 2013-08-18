#!/bin/bash

set -x


cd $DRC_BASE

cd software/externals
make -j4

cd ..
make -j4

cd ../ros_workspace
make -j4
