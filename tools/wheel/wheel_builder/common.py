# This file contains shared logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import argparse
import os
import re
import sys

# Location where most of the build will take place.
build_root = '/opt/drake-wheel-build'

# Location where testing of the wheel will take place.
test_root = '/opt/drake-wheel-test'

# Location where the wheel will be produced.
wheel_root = os.path.join(build_root, 'wheel')
wheelhouse = os.path.join(wheel_root, 'wheelhouse')

# Location of various scripts and other artifacts used to complete the build.
resource_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


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
    """
    Determines the complete name of the Drake wheel, given various individual
    bits such as the Drake version, Python version, and Python wheel platform.
    """
    vm = f'cp{python_version}'
    return f'drake-{wheel_version}-{vm}-{vm}-{wheel_platform}.whl'


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


def find_tests(*test_subdirs):
    """
    Returns a list of tests in the common directory and any subdirectories
    given as additional arguments.
    """
    all_tests = []
    for test_dir in ('', *test_subdirs):
        tests = []

        test_dir_full = os.path.join(resource_root, 'test', 'tests', test_dir)
        for test in os.listdir(test_dir_full):
            if not os.path.isdir(os.path.join(test_dir_full, test)):
                tests.append(os.path.join('tests', test_dir, test))

        where = f'subdirectory {test_dir!r}' if len(test_dir) else 'directory'
        assert len(tests), f'No tests were found in the test {where}!'

        all_tests += sorted(tests)

    return all_tests


def do_main(args, platform):
    """
    Entry point; performs the build using the given CLI arguments, platform,
    and resource root.

    The `platform` must be either a `linux` or `macos` object which provides
    platform-specific implementations of various operations necessary to
    complete the build.
    """

    # Work around `bazel run` changing the working directory; this is to allow
    # the user to pass in relative paths in a sane manner.
    real_cwd = os.environ.get('BUILD_WORKING_DIRECTORY')
    if real_cwd is not None:
        os.chdir(real_cwd)

    # Set up argument parser.
    parser = argparse.ArgumentParser(
        prog='build-wheels',
        description='Build the Drake PyPI wheel(s).')

    parser.add_argument(
        'version',
        help='PEP 440 version number with which to label the wheels')

    parser.add_argument(
        '-o', '--output-dir', metavar='DIR', default=os.path.realpath('.'),
        help='directory into which to extract wheels (default: %(default)r)')
    parser.add_argument(
        '-n', '--no-extract', dest='extract', action='store_false',
        help='build images but do not extract wheels')
    parser.add_argument(
        '--no-test', dest='test', action='store_false',
        help='build images but do not run tests')

    if platform is not None:
        platform.add_build_arguments(parser)
        platform.add_selection_arguments(parser)

    parser.add_argument(
        '--pep440', action='store_true',
        help='validate version number without building anything')

    # Parse arguments.
    options = parser.parse_args(args)
    if platform is not None:
        platform.fixup_options(options)

    if not _check_version(options.version):
        die(f'Version \'{options.version}\' does NOT conform to PEP 440')

    if options.pep440:
        print(f'Version \'{options.version}\' conforms to PEP 440')
        return

    if platform is not None:
        platform.build(options)
    else:
        die('Building wheels is not supported on this platform '
            f'(\'{sys.platform}\')')
    print('wheel_builder: SUCCESS')
