#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys


def extract_libraries(path):
    libs = []

    proc = subprocess.Popen(
        ['otool', '-L', path],
        stdout=subprocess.PIPE,
        text=True,
    )

    while True:
        line = proc.stdout.readline()
        if not line:
            break

        m = re.match('^\t(.*)[(][^)]+[)]\\s*$', line)
        if m is not None:
            libs.append(m.group(1).strip())

    return libs


def chlpath(path, libs, old, new):
    changes = []

    for lib in libs:
        if lib.startswith(old):
            changes += ['-change', lib, new + lib[len(old):]]

    if len(changes):
        subprocess.check_call(
            ['install_name_tool'] + changes + [path],
        )


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Change the prefix of dependent shared libraries.')
    parser.add_argument(
        'old',
        help='Old prefix to be replaced')
    parser.add_argument(
        'new',
        help='Replacement prefix')
    parser.add_argument(
        'library', nargs='+',
        help='Dynamic library to modify')

    # Parse arguments.
    options = parser.parse_args(args)

    # Get list of dependent libraries and modify their prefixes.
    for path in options.library:
        libs = extract_libraries(path)
        chlpath(path, libs=libs, old=options.old, new=options.new)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
