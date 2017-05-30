#!/bin/bash

# ./runall.sh

# Examples of environment variables to be set:
#   PREFIX="haswell-fma-"
#   CXX_FLAGS="-mfma"
#   CXX=clang++

# Options:
#   -up : enforce the recomputation of existing data, and keep best results as a merging strategy
#   -s  : recompute selected changesets only and keep bests

./run.sh gemm gemm_settings.txt $*
./run.sh lazy_gemm lazy_gemm_settings.txt $*
./run.sh gemv gemv_settings.txt $*
./run.sh gemvt gemv_settings.txt $*
./run.sh trmv_up gemv_square_settings.txt $*
./run.sh trmv_lo gemv_square_settings.txt $*
./run.sh trmv_upt gemv_square_settings.txt $*
./run.sh trmv_lot gemv_square_settings.txt $*
./run.sh llt gemm_square_settings.txt $*

