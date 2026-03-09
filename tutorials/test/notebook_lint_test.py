import sys

from python import runfiles

_PREAMBLE = """\
{
 "cells": [
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "# TITLE PLACEHOLDER\\n",
    "For instructions on how to run these tutorial notebooks, please see the \
[index](./index.ipynb).\\n"
"""


def _check_preamble(name, contents):
    """Every file except the index must open with a standard preamble."""
    if name == "tutorials/index.ipynb":
        return 0
    print(name)
    for i, expected_line in enumerate(_PREAMBLE.splitlines()):
        actual_line = contents[i]
        if "PLACEHOLDER" in expected_line:
            # This line need not match.
            continue
        if actual_line.rstrip(",") == expected_line.rstrip(","):
            # The line matches, modulo trailing commas; given how JSON lists
            # work, we must ignore trailing commas.
            continue
        print(f"{name}:{i + 1}:1: preamble is incorrect:", file=sys.stderr)
        print(f"   actual: {repr(actual_line)}", file=sys.stderr)
        print(f" expected: {repr(expected_line)}", file=sys.stderr)
        return 1
    return 0


def _check_matplotlib(name, contents):
    """Forbid %matplotlib notebook; it does not work on Deepnote."""
    for i, line in enumerate(contents):
        if "%matplotlib notebook" not in line:
            continue
        if '"# %matplotlib notebook' in line:
            # Don't complain about commented-out uses.
            continue
        print(f"{name}:{i + 1}:1: do not use %matplotlib", file=sys.stderr)
        return 1
    return 0


def _check_katex(name, contents):
    """Deepnote (and others) use KaTeX for LaTeX.
    $\\begin{aligned}math here \\end{aligned}$ works; `gathered` also works.
    The more typical `gather` and `align` do not.
    """
    for i, line in enumerate(contents):
        if "begin{align}" in line:
            print(f"{name}:{i + 1}:1: do not use 'align'", file=sys.stderr)
            print(" Re-write it to use 'aligned' instead.", file=sys.stderr)
            return 1
        if "begin{gather}" in line:
            print(f"{name}:{i + 1}:1: do not use 'gather'", file=sys.stderr)
            print(" Re-write it to use 'gathered' instead.", file=sys.stderr)
            return 1
    return 0


def _check_cell_outputs(name, contents):
    """In source control, all outputs should be cleared before pushing."""
    good_lines = [
        '   "execution_count": null,',
        '   "outputs": [],',
    ]
    for expected in good_lines:
        needle = expected.split(":")[0]
        for i, line in enumerate(contents):
            if not line.startswith(needle):
                # Not an outputs line.
                continue
            if line == expected:
                # A valid outputs line.
                continue
            print(f"{name}:{i + 1}:1: clear all outputs", file=sys.stderr)
            return 1
    return 0


def main():
    notebooks = sys.argv[1:]
    manifest = runfiles.Create()

    num_errors = 0
    for item in notebooks:
        filename = manifest.Rlocation(f"drake/tutorials/{item}")
        with open(filename, encoding="utf-8") as f:
            contents = f.read().splitlines()
        name = f"tutorials/{item}"
        num_errors += _check_preamble(name, contents)
        num_errors += _check_matplotlib(name, contents)
        num_errors += _check_katex(name, contents)
        num_errors += _check_cell_outputs(name, contents)

    if num_errors > 0:
        sys.exit(1)


assert __name__ == "__main__"
main()
