Multibody Benchmarks
--------------------

# acrobot

This is a simple benchmarking program that uses the acrobot plant to benchmark
the performance of various plant operations under autodiff.  It is used by
Drake developers to detect and avoid performance regressions.

# cassie

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

## Running an experiment

On the default supported platform, the following command will build code
and save result data to a user supplied directory, under relatively
controlled conditions.

    $ bazel run //multibody/benchmarking:cassie_experiment -- --output_dir=trial1

The script will attempt to reduce result variance by controlling cpu
throttling and by setting a cpu affinity mask.

It is still up to the user to make sure the machine is appropriate (not
a virtual machine, for example), and relatively unloaded. Close as many
running programs as is practical.

Optionally, it is possible to pass command line arguments through to the
`cassie` executable. See 'Configuring details of benchmark runs' below.

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

## Results data files

Note that the results output is an ongoing effort to record the
conditions at the time the experiment was run, but it is not currently
complete. Still missing:

 * git hash or similar source code version marker
 * platform information for unsupported platforms

## Configuring details of benchmark runs

In addition to the above forms, it is possible run the benchmark program
directly:

    $ bazel run //multibody/benchmarking:cassie [-- [ARGS...]]

or

    $ bazel-bin/multibody/benchmarking/cassie [ARGS...]

if the `:cassie` target is built. The direct forms may be useful
for combining debugging or profiling tools.

All of these forms take the same set of command line arguments, which
can select which cases are run, control iterations and repetitions,
alter output formatting, etc.

Documentation for command line arguments is here:
https://github.com/google/benchmark#command-line

# iiwa_relaxed_pos_ik

A benchmark for InverseKinematics.

# position_constraint

A benchmarks for PositionConstraint.
