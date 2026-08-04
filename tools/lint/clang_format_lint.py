import os
from pathlib import Path
import subprocess
import sys

import tools.lint.clang_format as clang_format_lib

# From https://bazel.build/reference/be/c-cpp#cc_library.srcs.
# Keep this list in sync with cpplint.bzl.
_SOURCE_EXTENSIONS = [
    source_ext
    for source_ext in """
.c
.cc
.cpp
.cxx
.c++
.C
.h
.hh
.hpp
.hxx
.inc
.inl
.H
""".split("\n")
    if len(source_ext)
]


def _is_cxx(filename):
    """Returns True only if filename is reasonable to clang-format."""
    root, ext = os.path.splitext(filename)
    _, penultimate_ext = os.path.splitext(root)

    # Don't clang-format CUDA files.
    if penultimate_ext == "cu":
        return False

    return ext in _SOURCE_EXTENSIONS


def _check_clang_format_idempotence(filename):
    clang_format = clang_format_lib.get_clang_format_path()
    current = Path(filename).read_text(encoding="utf-8")
    formatted = subprocess.check_output(
        [clang_format, "-style=file", filename],
        encoding="utf-8",
    )
    if current == formatted:
        return 0
    print(f"ERROR: {filename} needs clang-format")
    print(
        "note: fix via bazel-bin/tools/lint/clang-format "
        f"-style=file -i {filename}"
    )
    print(
        "note: if that program does not exist, you might need to compile it "
        "first: bazel build //tools/lint/..."
    )
    return 1


def main():
    """Checks that clang-format is idempotent on each path specified as a
    command-line argument.  Exit 1 if any of the paths are invalid or
    clang-format suggests any edits.  Otherwise exit 0.
    """
    total_errors = 0
    for filename in sys.argv[1:]:
        if not _is_cxx(filename):
            print("clang_format_lint.py: Skipping " + filename)
            continue
        print("clang_format_lint.py: Linting " + filename)
        total_errors += _check_clang_format_idempotence(filename)

    if total_errors == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
