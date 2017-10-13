#!/usr/bin/env python2

"""Rewrite the filenames given on the command line to obey formatting rules for
#include statements.  The only changes this script will make are to relocate
#include statements, and possibly some associated additions or removals of
blank lines near the #include blocks.

"""

import argparse
import os
import sys

from tools.lint.formatter import IncludeFormatter


def main():
    parser = argparse.ArgumentParser(
        prog='clang-format-includes',
        description=__doc__)
    parser.add_argument(
        'filenames', metavar='filename', type=str, nargs='*',
        help='full path to filename to reformat in place')
    parser.add_argument(
        '--all', action='store_true', default=False,
        help='reformat all first-party sources within the project')
    parser.add_argument(
        '--check-only', action='store_true', default=False,
        help='check if the file(s) are formatted correctly; do not edit')
    args = parser.parse_args()

    if args.all:
        # TODO(jwnimmer-tri) Consolidate this logic with the cpplint_wrapper
        # tree searching logic, including some way to unit test "all" search.
        extensions = ["cc", "h", "cpp"]
        pathnames = ["drake"]
        filenames = [
            os.path.join(dirpath, filename)
            for pathname in pathnames
            for dirpath, _, filenames in os.walk(pathname)
            for filename in filenames
            if os.path.splitext(filename)[1][1:] in extensions and
            "/third_party/" not in dirpath and
            "/matlab/" not in dirpath
        ]
    else:
        filenames = args.filenames

    num_errors = 0
    for filename in filenames:
        tool = IncludeFormatter(filename)
        tool.format_includes()
        if tool.is_same_as_original():
            continue
        if args.check_only:
            num_errors += 1
        else:
            tool.rewrite_file()

    if num_errors > 0:
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
