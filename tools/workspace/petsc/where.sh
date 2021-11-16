#!/bin/bash

set -e

[[ -n "$1" ]]
cd ~/jwnimmer-tri/drake/multibody/fixed_fem/dev/tmp/petsc/arch-linux-c-opt/obj
find . -name '*.o' | xargs -t -n1 objdump -t 2>&1 | egrep '(objdump|bss.*'"$1"')' | fgrep -B 1 bss | head -n 1 | sed -e 's#.*\./#src/#; s#\.o#.c#;'
