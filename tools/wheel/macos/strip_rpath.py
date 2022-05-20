"""
Remove RPATH commands from an artifact (binary or library).
"""

import argparse
import re
import subprocess
import sys

from drake.tools.install import otool


def filter_rpaths(paths, exclusions):
    result = []
    for p in paths:
        filtered = False
        for x in exclusions:
            if p.startswith(x):
                filtered = True
                break

        if not filtered:
            result.append(p)

    return result


def strip_rpaths(path, rpaths):
    """
    Remove RPATHs `rpaths` from binary/library `path`.
    """
    for rpath in rpaths:
        subprocess.check_call(
            ['install_name_tool', '-delete_rpath', rpath, path],
        )


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Strip RPATH(s) from a library.')
    parser.add_argument(
        '-x', '--exclude', metavar='PREFIX', action='append',
        help='leave any RPATH that starts with the specified prefix')
    parser.add_argument(
        'library',
        help='dynamic library to modify')

    # Parse arguments.
    options = parser.parse_args(args)

    # Extract RPATH entries from load commands.
    commands = otool.load_commands(options.library)
    rpaths = filter_rpaths(
        [c['path'] for c in commands if c['cmd'] == 'LC_RPATH'],
        options.exclude)

    # Remove the RPATH load commands.
    strip_rpaths(options.library, rpaths)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
