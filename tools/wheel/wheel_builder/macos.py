# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import glob
import os
import platform
import shutil
import subprocess

from .common import die, wheel_name
from .common import build_root, resource_root, wheelhouse


def _find_wheel(path, version):
    """
    Returns name of built wheel. Uses `glob` to find it, since trying to
    replicate the logic which determines the macOS platform name is not
    accessible and is very non-trivial to replicate.
    """
    pattern = wheel_name(
        python_version=''.join(platform.python_version_tuple()[:2]),
        wheel_version=version,
        wheel_platform='*')

    candidates = glob.glob(os.path.join(path, pattern))

    if len(candidates) != 1:
        die('Build was expected to produce exactly 1 wheel, '
            f'but {len(candidates)} were found!')

    return candidates[0]


def _assert_isdir(path, name):
    """
    Asserts that `path` is a directory. Shows an error otherwise, where `name`
    describes what path is being asserted.
    """
    if not os.path.isdir(path):
        die(f'{name} \'{path}\' is not a valid directory')


def _provision():
    """
    Prepares wheel build environment.
    """
    packages_path = os.path.join(resource_root, 'image', 'packages-macos')
    command = ['brew', 'bundle', f'--file={packages_path}', '--no-lock']
    subprocess.check_call(command)


def _test_wheel(path, env):
    """
    Runs the test script on the wheel at `path`.
    """
    test_script = os.path.join(resource_root, 'macos', 'test-wheel.sh')
    subprocess.check_call(['bash', test_script, path], env=env)


def build(options):
    """
    Builds wheel(s) with the provided options.
    """
    if options.extract:
        _assert_isdir(options.output_dir, 'Output location')

    _provision()

    # Sanitize the build/test environment.
    environment = os.environ.copy()
    environment.pop('PYTHONPATH')
    environment.pop('RUNFILES_MANIFEST_FILE')

    # Build the wheel.
    build_script = os.path.join(resource_root, 'macos', 'build-wheel.sh')
    build_command = ['bash', build_script]
    if options.incremental:
        build_command.append('--no-deps')
    build_command.append(options.version)

    subprocess.check_call(build_command, env=environment)

    # Find the built wheel and, if requested, test and/or extract it.
    wheel = _find_wheel(path=wheelhouse, version=options.version)

    if options.test:
        _test_wheel(wheel, env=environment)

    if options.extract:
        shutil.copy2(wheel, options.output_dir)

    if not options.keep_build:
        shutil.rmtree('/opt/vtk')
        shutil.rmtree('/opt/drake-dependencies')
        shutil.rmtree('/opt/drake')
        shutil.rmtree(build_root)


def add_build_arguments(parser):
    """
    Adds arguments that control the build.
    """
    parser.add_argument(
        '-k', '--keep-build', action='store_true',
        help='do not delete build tree after successful build')
    parser.add_argument(
        '--incremental', action='store_true',
        help='only build Drake itself '
             '(requires previously built dependencies)')


def add_selection_arguments(parser):
    """
    Adds arguments that control which wheel(s) to build.
    (No-op on macOS.)
    """
    pass  # macOS can only build one wheel.


def fixup_options(options):
    """
    Validates options and applies any necessary transformations.
    (No-op on macOS.)
    """
    pass  # Not needed on macOS.
