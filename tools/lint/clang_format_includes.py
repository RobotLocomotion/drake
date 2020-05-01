"""Rewrite the filenames given on the command line to obey formatting rules for
#include statements.  The only changes this script will make are to relocate
#include statements, and possibly some associated additions or removals of
blank lines near the #include blocks.
"""

import argparse
import os
import sys

from drake.tools.lint.formatter import IncludeFormatter
from drake.tools.lint.util import find_all_sources


def main(workspace_name="drake"):
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
        workspace_dir, relpaths = find_all_sources(workspace_name)
        extensions = ["cc", "h", "cpp"]
        filenames = [
            os.path.join(workspace_dir, relpath)
            for relpath in relpaths
            if os.path.splitext(relpath)[1][1:] in extensions
            and not relpath.startswith("third_party")
        ]
        print(f"This will reformat {len(filenames)} files "
              f"within {workspace_dir}")
        if input("Are you sure [y/N]? ") not in ["y", "Y"]:
            print("... canceled")
            sys.exit(1)
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
