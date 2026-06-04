"""Benchmarks the box_stacks example across a range of CENIC island-solver
thread counts, reporting the wall-clock simulation time and the speedup
relative to the first thread count tested.

Run it (after building) with, e.g.:

  bazel run //examples/multibody/box_stacks:benchmark_threads -- \\
      --threads=1,2,4,8 --num_stacks=16 --boxes_per_stack=8

Any flags other than --threads and --repeats are passed through to the
box_stacks binary, so you can choose the workload (number of piles, contact
model, accuracy, etc.). With no pass-through flags a default workload is used.
"""

import argparse
import os
import re
import subprocess
import sys

from python import runfiles

_WALL_RE = re.compile(r"Wall-clock simulation time:\s*([0-9.eE+-]+)\s*s")


def _find_binary():
    manifest = runfiles.Create()
    path = manifest.Rlocation("drake/examples/multibody/box_stacks/box_stacks")
    assert path, "Could not locate the box_stacks binary in runfiles."
    return path


def _default_threads():
    cpu = os.cpu_count() or 1
    counts = [1]
    n = 2
    while n < cpu:
        counts.append(n)
        n *= 2
    if cpu not in counts:
        counts.append(cpu)
    return counts


def _run_once(binary, num_threads, passthrough):
    env = dict(os.environ, OMP_NUM_THREADS=str(num_threads))
    args = [binary, *passthrough, f"--num_threads={num_threads}"]
    result = subprocess.run(
        args,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        encoding="utf-8",
        check=True,
    )
    match = _WALL_RE.search(result.stdout)
    if match is None:
        sys.stderr.write(result.stdout)
        raise RuntimeError("Could not parse wall-clock time from output.")
    return float(match.group(1))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--threads",
        default=None,
        help="Comma-separated list of thread counts to test. Defaults to "
        "powers of two up to the CPU count.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=3,
        help="Timed runs per thread count; the minimum is reported.",
    )
    args, passthrough = parser.parse_known_args()

    threads = (
        [int(t) for t in args.threads.split(",")]
        if args.threads
        else _default_threads()
    )

    binary = _find_binary()

    # Default workload (only when the user passes no example flags): enough
    # islands and per-island work to expose thread scaling.
    if not passthrough:
        passthrough = [
            "--num_stacks=16",
            "--boxes_per_stack=8",
            "--simulation_time=2.0",
            "--fixed_step=1",
            "--max_step_size=0.01",
        ]

    print(f"Binary: {binary}")
    print(f"Workload: {' '.join(passthrough)}")
    print(f"Repeats per thread count: {args.repeats}\n")

    # Warm up (build caches, page in the binary) before timing.
    _run_once(binary, threads[0], passthrough)

    baseline = None
    print(f"{'threads':>8}  {'best [s]':>10}  {'speedup':>8}")
    for num_threads in threads:
        best = min(
            _run_once(binary, num_threads, passthrough)
            for _ in range(args.repeats)
        )
        if baseline is None:
            baseline = best
        print(f"{num_threads:>8}  {best:>10.4f}  {baseline / best:>7.2f}x")


if __name__ == "__main__":
    main()
