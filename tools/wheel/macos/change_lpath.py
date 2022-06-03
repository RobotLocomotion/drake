"""
Given an input artifact (binary or library), apply a specified change to the
path prefix of all libraries to which the artifact is linked.
"""

import argparse
import re
import subprocess
import sys

from drake.tools.install import otool


def _chlpath(path, libs, old, new):
    """
    Builds and applies a set of changes to a library's load paths.

    Given the set of linked libraries `libs` of the library at `path`, for each
    linked library whose path starts with `old`, replace `old` with `new`.
    """
    changes = []

    for lib in libs:
        if lib.path.startswith(old):
            changes += ['-change', lib.path, new + lib.path[len(old):]]

    if len(changes):
        subprocess.check_call(
            ['install_name_tool'] + changes + [path],
        )


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Change the prefix of dependent shared libraries.')
    parser.add_argument(
        '--old', required=True,
        help='Old prefix to be replaced')
    parser.add_argument(
        '--new', required=True,
        help='Replacement prefix')
    parser.add_argument(
        'library', nargs='+',
        help='Dynamic library to modify')

    # Parse arguments.
    options = parser.parse_args(args)

    # Get list of dependent libraries and modify their prefixes.
    for path in options.library:
        libs = otool.linked_libraries(path)
        _chlpath(path, libs=libs, old=options.old, new=options.new)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
