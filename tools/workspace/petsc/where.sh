#!/bin/bash

set -e

[[ -n "$1" ]]
cd ~/jwnimmer-tri/drake/multibody/fixed_fem/dev/tmp/petsc/arch-linux-c-opt/obj

database=/home/jwnimmer/jwnimmer-tri/drake/multibody/fixed_fem/dev/tmp/petsc/objdump-database.txt
if [[ ! -e ${database} ]]; then
  find . -name '*.o' |
    xargs -t -n1 objdump -w -t 2>&1 |
    cat > ${database}
fi

cat ${database} |
  egrep '(^objdump|bss.*'"$1"'|text.*'"$1"'$)' |
  egrep -B 1 '\.(bss|text)' |
  head -n 1 |
  sed -e 's#.*\./#src/#; s#\.o#.c#;'
