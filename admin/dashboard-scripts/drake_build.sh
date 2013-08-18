#!/bin/bash

set -x


cd $DRAKE_BASE

make -j4
make test
