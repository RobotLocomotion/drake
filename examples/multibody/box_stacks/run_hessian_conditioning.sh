#!/bin/bash
# DIAGNOSTIC (uncommitted): measure and plot how constraint islands affect the
# conditioning of the ICF Hessian on the box_stacks example.
#
# Runs box_stacks twice in fixed-step mode -- once solving per-island, once
# solving the full problem (all one island) -- recording the Hessian condition
# number at every step, then plots a comparison.
#
# Usage (from the drake repo root):
#   examples/multibody/box_stacks/run_hessian_conditioning.sh [OUT_DIR] [EXTRA_FLAGS...]
#
# Any EXTRA_FLAGS are passed to both runs (e.g. --num_stacks=4 --simulation_time=1
# --boxes_per_stack=5). Both runs must use identical scene flags, which this
# script guarantees.
set -euo pipefail

OUT_DIR="${1:-/tmp/box_stacks_hessian}"
shift || true
EXTRA_FLAGS=("$@")

mkdir -p "${OUT_DIR}"

echo "Building box_stacks..."
bazel build //examples/multibody/box_stacks:box_stacks
BIN=bazel-bin/examples/multibody/box_stacks/box_stacks

echo "Running with islands ON..."
"${BIN}" --fixed_step --use_islands=true \
    --hessian_csv="${OUT_DIR}/islands_on.csv" "${EXTRA_FLAGS[@]}"

echo "Running with islands OFF (full solve)..."
"${BIN}" --fixed_step --use_islands=false \
    --hessian_csv="${OUT_DIR}/islands_off.csv" "${EXTRA_FLAGS[@]}"

echo "Plotting..."
python3 examples/multibody/box_stacks/plot_hessian_conditioning.py \
    --islands_on "${OUT_DIR}/islands_on.csv" \
    --islands_off "${OUT_DIR}/islands_off.csv" \
    --out "${OUT_DIR}/hessian_conditioning.png"

echo "Done. See ${OUT_DIR}/hessian_conditioning.png"
