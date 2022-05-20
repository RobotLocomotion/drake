# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import glob
import os
import platform
import shutil
import subprocess

from .common import die, wheel_name

# Location of various scripts and other artifacts used to complete the build.
# Must be set; normally by common.entry.
resource_root = None

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
    return glob.glob(os.path.join(path, pattern))[0]


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


def _test_wheel(path, env=None):
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

    _assert_isdir(options.build_root, 'Build root')

    _provision()

    # Set build root in the build/test environment.
    environment = os.environ.copy()
    environment['DRAKE_WHEELBUILD_PREFIX'] = options.build_root

    # Sanitize the build/test environment.
    environment.pop('PYTHONPATH')
    environment.pop('RUNFILES_MANIFEST_FILE')

    # Build the wheel.
    build_script = os.path.join(resource_root, 'macos', 'build-wheel.sh')

    subprocess.check_call(['bash', build_script, options.version],
                          env=environment)

    # Find the built wheel and, if requested, test and/or extract it.
    wheel = _find_wheel(
        path=os.path.join(options.build_root, 'wheel', 'wheelhouse'),
        version=options.version)

    if options.test:
        _test_wheel(wheel, env=environment)

    if options.extract:
        shutil.copy2(wheel, options.output_dir)


def add_build_arguments(parser):
    """
    Adds arguments that control the build.
    """
    default_root = os.environ.get('DRAKE_WHEELBUILD_PREFIX', '/')
    parser.add_argument(
        '-r', '--build-root', metavar='DIR', default=default_root,
        help='root directory for build trees (default: %(default)r)')


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
