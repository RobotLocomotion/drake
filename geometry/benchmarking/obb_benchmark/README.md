Obb benchmark
-------------

Conduct benchmarking experiment under relatively controlled conditions to
minimize runtime variations between repeated runs by controlling CPUs. 

## Supported experiments

General syntax:

    $ geometry/benchmarking/obb_benchmark/conduct_experiment [DIRECTORY] [ARGS...]

This is an example of how to run the benchmark:

    $ geometry/benchmarking/obb_benchmark/conduct_experiment \
      ~/project/ObbBench/record
      
You will get these files in ~/project/ObbBench/record directory.

    $ ls ~/project/ObbBench/record/
    compiler.txt  kernel.txt  os.txt  outputs.zip  results.json  summary.txt
    
The file summary.txt shows the statistics towards the end of the file.

    $ tail -n12 ~/project/ObbBench/record/summary.txt
    -------------------------------------------------------------------------------------
    Benchmark                                           Time             CPU   Iterations
    -------------------------------------------------------------------------------------
    BM_ObbHasOverlap_31_Cases_mean                   3413 ns         3413 ns            9
    BM_ObbHasOverlap_31_Cases_median                 3409 ns         3409 ns            9
    BM_ObbHasOverlap_31_Cases_stddev                 7.96 ns         7.97 ns            9
    BM_ObbHasOverlap_16_OverlapCases_mean            1920 ns         1920 ns            9
    BM_ObbHasOverlap_16_OverlapCases_median          1911 ns         1911 ns            9
    BM_ObbHasOverlap_16_OverlapCases_stddev          10.5 ns         10.5 ns            9
    BM_ObbHasOverlap_15_NonOverlapCases_mean         1551 ns         1551 ns            9
    BM_ObbHasOverlap_15_NonOverlapCases_median       1543 ns         1543 ns            9
    BM_ObbHasOverlap_15_NonOverlapCases_stddev       9.67 ns         9.67 ns            9

## Prepare for AVX Experiments

The above data is from the default build flags on my puget-183882-03.
Next I switched "*conduct_experiment*" to these build flags:

    ./tools/performance/benchmark_tool conduct_experiment \
        -b=--compilation_mode=opt \
        -b=--copt=-g \
        -b=--cxxopt=-march=broadwell \
        $TARGET "$OUTPUT_DIR" -- "$@"

Then, I got:

    $ tail -n12 ~/project/ObbBench/2021-02-05_1400_broadwell/summary.txt 
    -------------------------------------------------------------------------------------
    Benchmark                                           Time             CPU   Iterations
    -------------------------------------------------------------------------------------
    BM_ObbHasOverlap_31_Cases_mean                   3498 ns         3498 ns            9
    BM_ObbHasOverlap_31_Cases_median                 3490 ns         3490 ns            9
    BM_ObbHasOverlap_31_Cases_stddev                 12.7 ns         12.7 ns            9
    BM_ObbHasOverlap_16_OverlapCases_mean            1874 ns         1874 ns            9
    BM_ObbHasOverlap_16_OverlapCases_median          1875 ns         1875 ns            9
    BM_ObbHasOverlap_16_OverlapCases_stddev          9.21 ns         9.20 ns            9
    BM_ObbHasOverlap_15_NonOverlapCases_mean         1512 ns         1512 ns            9
    BM_ObbHasOverlap_15_NonOverlapCases_median       1504 ns         1504 ns            9
    BM_ObbHasOverlap_15_NonOverlapCases_stddev       9.11 ns         9.11 ns            9

## More info

Google Benchmark Documentation
https://github.com/google/benchmark#benchmark