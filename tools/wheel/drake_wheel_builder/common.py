# This file contains shared logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import argparse
import os
import re
import sys

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def gripe(message):
    """
    Prints a message to stderr.
    """
    print(message, file=sys.stderr)


def die(message, result=1):
    """
    Prints a message to stderr and aborts.
    """
    gripe(message)
    sys.exit(result)


def wheel_name(python_version, wheel_version, wheel_platform):
    vm = f'cp{python_version}'
    if python_version < '38':
        abi = f'{vm}m'
    else:
        abi = f'{vm}'
    return f'drake-{wheel_version}-{vm}-{abi}-{wheel_platform}.whl'


def check_version(version):
    """Returns True iff the given version string matches PEP 440."""
    return re.match(
        r'^([1-9][0-9]*!)?(0|[1-9][0-9]*)'
        r'(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?'
        r'(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?'
        r'([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$',
        version) is not None


def entry(args, platform, resource_root):
    platform.resource_root = resource_root

    # Set up argument parser.
    parser = argparse.ArgumentParser(
        description='Build the Drake PyPI wheel(s).')

    parser.add_argument(
        'version',
        help='PEP 440 version number with which to label the wheels')

    parser.add_argument(
        '-o', '--output-dir', metavar='DIR', default=os.path.realpath('.'),
        help='directory into which to extract wheels (default: .)')
    parser.add_argument(
        '-n', '--no-extract', dest='extract', action='store_false',
        help='build images but do not extract wheels')

    platform.add_build_arguments(parser)

    parser.add_argument(
        '-t', '--test', action='store_true',
        help='run tests on wheels')

    platform.add_selection_arguments(parser)

    # Parse arguments.
    options = parser.parse_args(args)
    platform.fixup_options(options)

    if not check_version(options.version):
        gripe(f'Version \'{options.version}\' does not conform to PEP 440')
        sys.exit(1)

    platform.build(options)
