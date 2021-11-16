g++ -fPIC -Wall -Wwrite-strings -Wno-strict-aliasing -Wno-unknown-pragma -fstack-protector -fvisibility=hidden -O2 -march=haswell \
-I/dir/to/petsc/include \
-I/dir/to/petsc/arch-linux-c-opt/include \
-I/usr/include/eigen3  \
petsc_benchmark.cc \
-Wl,-rpath,/dir/to/petsc/arch-linux-c-opt/lib -L/dir/to/petsc/arch-linux-c-opt/lib \
-Wl,-rpath,/usr/lib/gcc/x86_64-linux-gnu/7 -L/usr/lib/gcc/x86_64-linux-gnu/7  \
-lpetsc -llapack -lblas \
-isystem /dir/to/google_benchmark/include -L/dir/to/google_benchmark/build/src -lbenchmark -lpthread \
-o petsc_benchmark  \
&& ./petsc_benchmark
