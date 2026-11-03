Multibody Benchmarks
--------------------

# acrobot

This is a simple benchmarking program that uses the acrobot plant to benchmark
the performance of various plant operations under autodiff.  It is used by
Drake developers to detect and avoid performance regressions.

# anchored_geometry

Measures the cost of collision geometry that is fixed in the world but is
registered on bodies welded to the world (as the Parser does for static models)
against the same geometry registered on the world body (anchored geometry): a
grid of static boxes with one free box resting on it, timing the point-pair
contact query and a discrete simulation step for growing numbers of boxes, as
well as building the model (registration plus Finalize), where the collision
filter's pairwise storage is paid for.

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

# hydroelastic_clutter

Measures hydroelastic-with-fallback contact on a cluttered, procedurally
generated desk scene, with no model files. The scene is built to be expensive in
the same ways as a real cluttered tabletop: anchored furniture made of many
small, oriented rigid pieces (many geometries that are near, but not in,
contact), and free manipulands that each have one compliant convex hull plus
several rigid convex parts (so that there is a mix of compliant-rigid,
compliant-compliant, and point-pair fallback contact). All geometry is
generated in code with a platform-independent random sequence. All contact
materials use a Hunt-Crossley dissipation of 1 s/m.

Two things are timed, on the scene after it has settled: the geometry query
`ComputeContactSurfacesWithFallback()` alone (`ContactSurfaces`), and one full
discrete step including contact-pair assembly and the SAP solve
(`SimulationStep`). The Args are the number of columns of free bodies and the
edge length (mm) of the desk pieces:

  - `3/60`: nv = 36 with about 20 hydroelastic contacts. A tiny case that
    exists only for the smoke test.
  - `12/60`: nv = 144 with about 80 hydroelastic contacts and a few
    point-pair contacts. The reference case.
  - `12/250`: the same free bodies, sticks and initial poses on a coarser desk
    (about 40 hydroelastic contacts; fewer, larger contacting pairs); useful
    for telling apart optimizations that scale with the number of small
    contacting pairs.
  - `64/60`: nv = 768 with about 450 hydroelastic contacts; useful for costs
    that scale with the number of degrees of freedom (e.g., Jacobian assembly).

The reported counters `nv`, `hydro_contacts`, and `point_pair_contacts` give the
size of each problem, and `settled_max_speed` (the largest generalized velocity
after the warmup) shows how close to rest the scene is. Building and settling a
scene takes seconds, and Google Benchmark repeats that while it searches for an
iteration count; pass `--benchmark_min_time=<N>x` to bound the number of runs.
The smoke test fails if the scene has no contacts at all.

The desk tiles, the sticks and the free bodies come from independent random
sequences, so changing the desk piece size changes nothing else. As with the
other benchmarks here, wall-clock times are specific to a machine; prefer to
compare before and after on the same machine (for example, with instruction
counts from `perf stat`, which are far less noisy than times).

# iiwa_relaxed_pos_ik

A benchmark for InverseKinematics.

# position_constraint

A benchmarks for PositionConstraint.
