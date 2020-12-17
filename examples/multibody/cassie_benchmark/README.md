Cassie benchmark
----------------

This is a real-world example of a medium-sized robot with timing
tests for calculating its mass matrix, inverse dynamics, and
forward dynamics and their AutoDiff derivatives.

This gives us a straightforward way to measure local,
machine-specific, improvements in these basic multibody calculations
in Drake.

This program does not serve as a performance regression test as it has
no way to normalize for changes in hardware or other environmental
factors. Its primary purpose is to serve as a platform where
performance can be measured, code tweaked, and performance measured
again. Improvements observed on this benchmark on a particular machine
do not guarantee improvements on other robots or other machines.

## Supported experiments

On the default supported platform, the following command will build code
and save result data to a user supplied directory, under relatively
controlled conditions.

    $ examples/multibody/cassie_benchmark/conduct_experiment [DIRECTORY] [ARGS...]

The `conduct_experiment` script will attempt to reduce result variance
by controlling cpu throttling and by setting a cpu affinity mask.

It is still up to the user to make sure the machine is appropriate (not
a virtual machine, for example), and relatively unloaded. Close as many
running programs as is practical.

Optionally, it is possible to pass command line arguments through to the
`cassie_bench` executable. See 'Configuring details of benchmark runs'
below.

## Comparing experiment data

Since experiment results are highly machine dependent, any stored
performance data is limited in usefulness for comparisons. The current
best practice recommendation is to only compare results taken from the
same machine and software environment, and to carefully track which
version(s) of drake software were measured.

## Storing experiment data

Given the limitations of benchmark data, storing data is only useful as
very general guidance to expected performance. Example data and
discussion of methods are being tracked in Github issue #13902.

## Experiment details

The following command will perform a benchmark experiment (with
environment controls) and collect context information. It is not
confined to a specific platform, like `conduct_experiment`, and it
controls fewer environment conditions.

    $ bazel run //examples/multibody/cassie_benchmark:record_results [-- [ARGS...]]

For best results, some conditions will need to be manually
controlled. These include:

* load average -- close as many other programs as practical
* cpu throttling -- varies with platform
  * Linux: https://github.com/google/benchmark#disabling-cpu-frequency-scaling

The `:record_results` target does try to mitigate costs of rescheduling
to a different processor (Linux only), by setting a processor
affinity. However, it does not do full-featured processor isolation, so
it is still important to limit the number of running programs. As of
this writing, the benchmark is assigned to processor #0; it may be worth
monitoring the system to ensure that processor will be fully available
to the experiment.

Optionally, it is possible to pass command line arguments through to the
`cassie_bench` executable. See 'Configuring details of benchmark runs'
below.

## Results data files

Bazel buries the results deep in its output tree. To copy them
somewhere more convenient, use this command outside of Bazel:

    $ examples/multibody/cassie_benchmark/copy_results_to [DIRECTORY]

Note that the results output is an ongoing effort to record the
conditions at the time the experiment was run, but it is not currently
complete. Still missing:

 * git hash or similar source code version marker
 * platform information for unsupported platforms

## Configuring details of benchmark runs

In addition to the above forms, it is possible run the benchmark program
directly:

    $ bazel run //examples/multibody/cassie_benchmark:cassie_bench [-- [ARGS...]]

or

    $ bazel-bin/examples/multibody/cassie_benchmark/cassie_bench [ARGS...]

if the `:cassie_bench` target is built. The direct forms may be useful
for combining debugging or profiling tools.

All of these forms take the same set of command line arguments, which
can select which cases are run, control iterations and repetitions,
alter output formatting, etc.

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line

