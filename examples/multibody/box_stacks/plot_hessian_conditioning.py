"""DIAGNOSTIC (uncommitted): plot Hessian condition number over time, comparing
the constraint-island solve against the no-island (full Hessian) solve.

This reads the CSVs written by box_stacks --hessian_csv (fixed-step mode only)
and plots condition number vs. time. With islands enabled each step has one
Hessian per island, so we show the per-island spread (min--max band) and the max
(the island that is hardest to solve). Without islands each step has a single,
much larger Hessian.

Typical workflow (run from the drake repo root):

    bazel build //examples/multibody/box_stacks:box_stacks
    BIN=bazel-bin/examples/multibody/box_stacks/box_stacks
    $BIN --fixed_step --use_islands=true  --hessian_csv=/tmp/islands_on.csv
    $BIN --fixed_step --use_islands=false --hessian_csv=/tmp/islands_off.csv
    python3 examples/multibody/box_stacks/plot_hessian_conditioning.py \
        --islands_on /tmp/islands_on.csv \
        --islands_off /tmp/islands_off.csv \
        --out /tmp/hessian_conditioning.png

Both runs must use identical scene flags (the defaults here) so the comparison
is apples-to-apples.
"""

import argparse
import csv
from collections import defaultdict

import numpy as np
import matplotlib

matplotlib.use("Agg")  # File output; no interactive backend needed.
import matplotlib.pyplot as plt  # noqa: E402


def load(path):
    """Returns (times, conds, islands) arrays for one CSV, where each row is one
    recorded Hessian. `islands` is the per-step island count from the file."""
    times, conds, num_islands = [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            times.append(float(row["time"]))
            conds.append(float(row["condition_number"]))
            num_islands.append(int(row["num_islands"]))
    return np.array(times), np.array(conds), np.array(num_islands)


def per_step(times, values, reducer):
    """Groups `values` by time and applies `reducer` (e.g. np.max) per group.
    Returns sorted (unique_times, reduced_values)."""
    groups = defaultdict(list)
    for t, v in zip(times, values):
        groups[t].append(v)
    ts = np.array(sorted(groups))
    return ts, np.array([reducer(groups[t]) for t in ts])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--islands_on", default="/tmp/islands_on.csv",
                        help="CSV from a --use_islands=true run.")
    parser.add_argument("--islands_off", default="/tmp/islands_off.csv",
                        help="CSV from a --use_islands=false run.")
    parser.add_argument("--out", default="/tmp/hessian_conditioning.png",
                        help="Output image path.")
    parser.add_argument("--show", action="store_true",
                        help="Also open an interactive window.")
    args = parser.parse_args()

    on_t, on_c, on_n = load(args.islands_on)
    off_t, off_c, _ = load(args.islands_off)

    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(9, 7), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]})

    # --- Condition number panel ---
    # No-island baseline: one (large) Hessian per step.
    off_ts, off_max = per_step(off_t, off_c, np.max)
    ax.plot(off_ts, off_max, "-o", color="C3", lw=2, ms=4,
            label="No islands (full Hessian)")

    # Island solve: spread across islands, plus the worst island per step.
    on_lo_t, on_lo = per_step(on_t, on_c, np.min)
    on_hi_t, on_hi = per_step(on_t, on_c, np.max)
    ax.fill_between(on_hi_t, on_lo, on_hi, color="C0", alpha=0.25,
                    label="Islands (min--max over islands)")
    ax.plot(on_hi_t, on_hi, "-o", color="C0", lw=2, ms=4,
            label="Islands (max over islands)")
    # Faint scatter of every island's condition number.
    ax.scatter(on_t, on_c, s=6, color="C0", alpha=0.15, linewidths=0)

    ax.set_yscale("log")
    ax.set_ylabel("Hessian condition number  (λ_max / λ_min)")
    ax.set_title("ICF Hessian conditioning: constraint islands vs. full solve\n"
                 "(box_stacks, fixed step)")
    ax.grid(True, which="both", ls=":", alpha=0.5)
    ax.legend(loc="best")

    # --- Island count panel for context ---
    n_ts, n_vals = per_step(on_t, on_n, lambda x: x[0])
    ax2.plot(n_ts, n_vals, "-o", color="C0", lw=1.5, ms=3)
    ax2.set_ylabel("# islands")
    ax2.set_xlabel("time [s]")
    ax2.grid(True, ls=":", alpha=0.5)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"Wrote {args.out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
