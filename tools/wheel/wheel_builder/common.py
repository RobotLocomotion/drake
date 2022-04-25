# This file contains shared logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import argparse
import os
import re
import sys


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


def _check_version(version):
    """
    Returns True iff the given version string matches PEP 440.
    """
    return re.match(
        r'^([1-9][0-9]*!)?(0|[1-9][0-9]*)'
        r'(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?'
        r'(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?'
        r'([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$',
        version) is not None


def do_main(args, platform):
    """
    Entry point; performs the build using the given CLI arguments, platform,
    and resource root.

    The `platform` must be either a `linux` or `macos` object which provides
    platform-specific implementations of various operations necessary to
    complete the build.
    """
    module_dir = os.path.dirname(os.path.realpath(__file__))
    platform.resource_root = os.path.dirname(module_dir)

    # Set up argument parser.
    parser = argparse.ArgumentParser(
        prog='build-wheels',
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

    if not _check_version(options.version):
        die(f'Version \'{options.version}\' does not conform to PEP 440')

    platform.build(options)
