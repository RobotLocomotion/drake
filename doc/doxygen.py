#!/usr/bin/env python2

"""Command-line tool to generate Drake's Doxygen content.

See drake/doc/documentation_instructions.rst for instructions and usage hints.
"""

from __future__ import print_function

import os
import sys

# This file is a legacy stub that launches the bazel-compiled doxygen wrapper.


def main():
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    drake_dir = os.path.dirname(cur_dir)
    binary = os.path.join(drake_dir, "bazel-bin/doc/doxygen")
    if not os.path.exists(binary):
        print("error: You must first run a bazel build //doc:doxygen")
        return 1
    print("warning: doxygen.py is deprecated; use bazel-bin/doc/doxygen")
    os.execv(binary, [binary] + sys.argv[1:])


if __name__ == '__main__':
    main()
