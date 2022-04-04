#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys


def extract_load_commands(path):
    commands = []
    command = None

    proc = subprocess.Popen(
        ['otool', '-l', path],
        stdout=subprocess.PIPE,
        text=True,
    )

    while True:
        line = proc.stdout.readline()
        if not line:
            break

        if line.startswith('Load command'):
            if command is not None and len(command):
                commands.append(command)

            command = {}

        elif line.startswith(' ') and command is not None:
            kv = line.lstrip().split(' ', 1)
            command[kv[0]] = kv[1].strip()

    return commands


def extract_rpath(load_command):
    return re.sub(' +[(]offset [0-9]+[)]$', '', load_command['path'])


def strip_rpaths(path, rpaths):
    for rpath in rpaths:
        if not rpath.startswith('/usr'):
            subprocess.check_call(
                ['install_name_tool', '-delete_rpath', rpath, path],
            )


def main(args):
    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Strip RPATH from a library.')
    parser.add_argument(
        'library',
        help='Dynamic library to modify')

    # Parse arguments.
    options = parser.parse_args(args)

    # Extract RPATH entries from load commands.
    commands = extract_load_commands(options.library)
    rpaths = [extract_rpath(c) for c in commands if c['cmd'] == 'LC_RPATH']

    # Remove the RPATH load commands.
    strip_rpaths(options.library, rpaths)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    main(sys.argv[1:])
