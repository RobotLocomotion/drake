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

    $ tail ~/project/ObbBench/record/summary.txt 
      L1 Instruction 32 KiB (x24)
      L2 Unified 256 KiB (x24)
      L3 Unified 30720 KiB (x2)
    Load Average: 0.41, 0.29, 0.53
    --------------------------------------------------------------------------
    Benchmark                                Time             CPU   Iterations
    --------------------------------------------------------------------------
    Benchmark_Obb_HasOverlap_mean         3642 ns         3642 ns            9
    Benchmark_Obb_HasOverlap_median       3644 ns         3644 ns            9
    Benchmark_Obb_HasOverlap_stddev       5.41 ns         5.42 ns            9

