#!/bin/sh

# A helper script used by python_lint.bzl that wraps `ruff format --check` with
# Drake-specific advice about how to fix the lint.

set -eu

# Loop over the files one by one so that we can give specific feedback.
needs_format=""
exitcode=0
for file in "$@"; do
  if ! ../ruff_prebuilt+/ruff format --check --quiet "${file}"; then
    needs_format="${needs_format} ${file}"
    exitcode=1
  fi
done

if test -n "${needs_format}"; then
  echo "ERROR: One or more files need ruff format" 1>&2
  echo "note: fix via: bazel run -- //tools/lint:ruff format${needs_format}" 1>&2
fi

exit ${exitcode}
