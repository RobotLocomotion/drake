"""Benchmarks the box_stacks example across a range of CENIC island-solver
thread counts, reporting the wall-clock simulation time and the speedup
relative to the first thread count tested.

Run it (after building) with, e.g.:

  bazel run //examples/multibody/box_stacks:benchmark_threads -- \\
      --threads=1,2,4,8 --num_stacks=16 --boxes_per_stack=8

Any flags other than --threads, --repeats, and --plot are passed through to the
box_stacks binary, so you can choose the workload (number of piles, contact
model, accuracy, etc.). With no pass-through flags a default workload is used.

Pass --plot (optionally with a path) to write a wall-time-vs-threads figure,
which also shows the no-islands baseline as a dotted horizontal line.
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


def _run_once(binary, num_threads, passthrough, extra_args=()):
    env = dict(os.environ, OMP_NUM_THREADS=str(num_threads))
    args = [binary, *passthrough, f"--num_threads={num_threads}", *extra_args]
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


# Friendlier display names for the workload flags that show up in the plot
# title; flags not listed fall back to their name with underscores replaced.
_FLAG_LABELS = {
    "num_stacks": "stacks",
    "boxes_per_stack": "boxes/stack",
    "stack_spacing": "spacing",
    "simulation_time": "sim time",
    "max_step_size": "max step",
    "fixed_step": "fixed step",
    "accuracy": "accuracy",
}


def _format_workload(passthrough):
    """Renders the pass-through flags into a readable, comma-separated string
    for a plot title (e.g. "stacks=9, boxes/stack=10, sim time=2.0")."""
    parts = []
    for flag in passthrough:
        token = flag.lstrip("-")
        if "=" in token:
            key, value = token.split("=", 1)
        else:
            key, value = token, None
        label = _FLAG_LABELS.get(key, key.replace("_", " "))
        parts.append(label if value is None else f"{label}={value}")
    return ", ".join(parts)


def _make_plot(threads, times, no_island_time, passthrough, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    ax.plot(threads, times, marker="o", label="islands")
    ax.axhline(
        no_island_time,
        linestyle=":",
        color="gray",
        label="no islands",
    )
    ax.set_xlabel("number of threads")
    ax.set_ylabel("wall-clock simulation time [s]")
    ax.set_xticks(threads)
    workload = _format_workload(passthrough)
    title = "Box stacks thread scaling"
    if workload:
        title += f"\n{workload}"
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    print(f"\nWrote plot to {path}")


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
    parser.add_argument(
        "--plot",
        nargs="?",
        const="benchmark_threads.png",
        default=None,
        help="Write a wall-time-vs-threads plot to this path (defaults to "
        "benchmark_threads.png). The no-islands baseline is shown as a "
        "dotted horizontal line.",
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
            "--num_stacks=9",
            "--boxes_per_stack=10",
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
    times = []
    print(f"{'threads':>8}  {'best [s]':>10}  {'speedup':>8}")
    for num_threads in threads:
        best = min(
            _run_once(binary, num_threads, passthrough)
            for _ in range(args.repeats)
        )
        if baseline is None:
            baseline = best
        times.append(best)
        print(f"{num_threads:>8}  {best:>10.4f}  {baseline / best:>7.2f}x")

    # No-islands baseline (the pre-island path, where --num_threads is ignored).
    no_island_time = min(
        _run_once(binary, 1, passthrough, extra_args=["--use_islands=false"])
        for _ in range(args.repeats)
    )
    print(f"\n{'no islands':>8}  {no_island_time:>10.4f} s")

    if args.plot is not None:
        _make_plot(threads, times, no_island_time, passthrough, args.plot)


if __name__ == "__main__":
    main()
