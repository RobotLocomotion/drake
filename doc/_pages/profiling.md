---
title: Running Profiling Tools
---

This page contains tips and tricks for capturing and understanding profiling
reports.

Some other good references are:
- [Callgrid documentation](https://valgrind.org/docs/manual/cl-manual.html)
- [Kcachegrind documentation](https://kcachegrind.github.io/html/Home.html)
- [Blog post](https://baptiste-wicht.com/posts/2011/09/profile-c-application-with-callgrind-kcachegrind.html)

# Installation

Install `valgrind`, `kcachegrind`, and `graphviz`. For example on Ubuntu:
```
sudo apt-get install valgrind kcachegrind graphviz
```

# Compiling your example program

When profiling, you should use a representative example program.

Compile your example program with debug symbols (line numbers) enabled, but
still in optimized mode (`-O2`). That will allow you to drill down into
line-by-line performance costs, while maintainaing representative
performance. These builds are usually termed as "release with debug info".
This is different than "debug builds" which typically have compiler
optimizations disabled (`-O0`).

In Bazel, build like this:

```
bazel build -c opt --copt=-g //foo/bar:example
```

In CMake, configure like this:

```
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

# Capturing a trace

Run `valgrind` with `callgrind` to get profiling data. For example:

```
valgrind --tool=callgrind bazel-bin/foo/bar/example
```

Each time this is run, it will create an output file file named
`callgrind.out.${PID}`. Typically you'd capture just one trace and inspect it
on its own, but in advanced uses you can combine multiple traces.

Oftentimes the call patterns in Drake will be difficult to view under the
default instrumentation settings, so we recommend running with extra flags.
These flags will slow down data collection, but are usually worth it:

```
valgrind --tool=callgrind \
  --separate-callers=10 \
   bazel-bin/foo/bar/example
```

If you're trying to micro-optimize a function, use `--dump-instr=yes` to see
per-instruction costs in the object code disassembly.

# Viewing the trace

Run `kcachegrind` to analyze profiling data. For example:

```
kcachegrind callgrind.out.19482
```

# Profiling Google Benchmark executables

[Google Benchmark](https://github.com/google/benchmark) throttles the number of
iterations it runs on each benchmark, which will skew the performance metrics
you see in the profiler. [You may want to manually specify the number of
iterations to run on each
benchmark](https://stackoverflow.com/questions/61843343/how-to-special-case-the-number-of-iterations-in-google-benchmark).
