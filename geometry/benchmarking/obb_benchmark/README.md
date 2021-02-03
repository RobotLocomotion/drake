Obb benchmark
-------------

Conduct benchmarking experiment on Obb::HasOverlap() under relatively controlled
conditions to minimize runtime variations between repeated runs by controlling
CPUs.

## Supported experiments

General syntax:

    $ geometry/benchmarking/obb_benchmark/conduct_experiment [DIRECTORY] [ARGS...]

This is an example of how to run the benchmark:

    $ geometry/benchmarking/obb_benchmark/conduct_experiment \
      ~/project/ObbBench/record

The directory will be created if it did not exist before. If it exists before,
the data files will be overwritten.

You will get these files in ~/project/ObbBench/record directory.

    $ ls ~/project/ObbBench/record/
    compiler.txt  kernel.txt  os.txt  outputs.zip  results.json  summary.txt

The file summary.txt shows the statistics near the end of the file.

    $ tail -n18 ~/project/ObbBench/record/summary.txt
      ------------------------------------------------------------------------------------
      Benchmark                                          Time             CPU   Iterations
      ------------------------------------------------------------------------------------
      ObbHasOverlap/All_31_Cases_mean                3440 ns         3440 ns            9
      ObbHasOverlap/All_31_Cases_median              3439 ns         3439 ns            9
      ObbHasOverlap/All_31_Cases_stddev              2.84 ns         2.85 ns            9
      ObbHasOverlap/All_31_Cases_min                 3437 ns         3436 ns            9
      ObbHasOverlap/All_31_Cases_max                 3445 ns         3445 ns            9
      ObbHasOverlap/Overlap_16_Cases_mean            1920 ns         1920 ns            9
      ObbHasOverlap/Overlap_16_Cases_median          1920 ns         1920 ns            9
      ObbHasOverlap/Overlap_16_Cases_stddev         0.580 ns        0.596 ns            9
      ObbHasOverlap/Overlap_16_Cases_min             1919 ns         1919 ns            9
      ObbHasOverlap/Overlap_16_Cases_max             1921 ns         1921 ns            9
      ObbHasOverlap/NonOverlap_15_Cases_mean         1575 ns         1575 ns            9
      ObbHasOverlap/NonOverlap_15_Cases_median       1575 ns         1575 ns            9
      ObbHasOverlap/NonOverlap_15_Cases_stddev       3.05 ns         3.01 ns            9
      ObbHasOverlap/NonOverlap_15_Cases_min          1571 ns         1572 ns            9
      ObbHasOverlap/NonOverlap_15_Cases_max          1583 ns         1583 ns            9


## Prepare for experiments on broadwell microarchitecture

"*conduct_experiment*" use these build flags to experiment with broadwell
 microarchitecture.

    $ tail -n5 geometry/benchmarking/obb_benchmark/conduct_experiment 
    ./tools/performance/benchmark_tool conduct_experiment \
        -b=--compilation_mode=opt \
        -b=--copt=-g \
        -b=--cxxopt=-march=broadwell \
        $TARGET "$OUTPUT_DIR" -- "$@"

If you want to benchmark another architecture, you can change the option
`-march=broadwell` to another one in the file `conduct_experiment`.

Notice that the argument `conduct_experiment` to the `benchmark_tool` refers
to its subcommand. It is not a recursive call back to the script
`obb_benchmark/conduct_experiment` in this directory.

## More info

Google Benchmark Documentation
https://github.com/google/benchmark#benchmark

