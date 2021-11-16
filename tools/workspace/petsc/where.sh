#!/bin/bash

set -e

[[ -n "$1" ]]
cd ~/jwnimmer-tri/drake/multibody/fixed_fem/dev/tmp/petsc/arch-linux-c-opt/obj
find . -name '*.o' |
  xargs -t -n1 objdump -w -t 2>&1 |
  egrep '(^objdump|bss.*'"$1"'|text.*'"$1"')' |
  egrep -B 1 '\.(bss|text)' |
  head -n 1 |
  sed -e 's#.*\./#src/#; s#\.o#.c#;'
