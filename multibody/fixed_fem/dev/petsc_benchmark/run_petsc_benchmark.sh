set -ex
PETSC=/home/jwnimmer/jwnimmer-tri/drake/multibody/fixed_fem/dev/tmp/petsc
GBENCH=/home/jwnimmer/jwnimmer-tri/drake/multibody/fixed_fem/dev/tmp/benchmark/install
g++ -fPIC -Wall -Wwrite-strings -Wno-strict-aliasing -Wno-unknown-pragma -fstack-protector -fvisibility=hidden -O2 -march=haswell \
-I${PETSC}/include \
-I${PETSC}/arch-linux-c-opt/include \
-I/usr/include/eigen3  \
petsc_benchmark.cc \
-Wl,-rpath,${PETSC}/arch-linux-c-opt/lib -L${PETSC}/arch-linux-c-opt/lib \
-Wl,-rpath,/usr/lib/gcc/x86_64-linux-gnu/7 -L/usr/lib/gcc/x86_64-linux-gnu/7  \
-lpetsc -llapack -lblas \
-isystem ${GBENCH}/include -L${GBENCH}/lib -lbenchmark -lpthread \
-o petsc_benchmark  \
&& ./petsc_benchmark
