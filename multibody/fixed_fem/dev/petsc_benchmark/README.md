Experimental PETSc benchmark.
================================================================================
 
A small benchmark to illustrate typical use of PETSc in Drake's FEM solver.

To run the benchmark:
1. Follow instructions from
[https://github.com/google/benchmark](https://github.com/google/benchmark) to
download and build Google benchmark.
2. Get PETSc from 
[https://github.com/petsc/petsc](https://github.com/petsc/petsc).
3. Configure PETSc with config script PETSc provides with the folloing flags
`./configure --with-debugging=no --COPTFLAGS="-O2" --CXXOPTFLAGS="-O2"
 --with-mpi=0 --with-fc=0 --with-sowing=0 --with-x=0 --with-pthread=0`.
4. Build PETSc with
`make PETSC_DIR=/dir/to/petsc PETSC_ARCH=arch-linux-c-opt all`.
5. Change `/dir/to/petsc/` and `/dir/to/google/benchmark` in
`run_petsc_benchmark.sh` to actual directories to PETSc and Google benchmark.
6. Run `run_petsc_benchmark.sh`.

