# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import os
import platform
import re
import shutil
import subprocess

from .common import die, wheel_name

resource_root = None

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def find_wheel(path):
    pattern = wheel_name(
        python_version=''.join(platform.python_version_tuple()[:2]),
        wheel_version=options.version,
        wheel_platform='*')
    return glob.glob(os.path.join(path, pattern))[0]


def assert_isdir(path, name):
    if not os.path.isdir(path):
        die(f'{name} \'{path}\' is not a valid directory')


def provision():
    packages_path = os.path.join(resource_root, 'image', 'packages-macos')
    command = ['brew', 'bundle', f'--file={packages_path}', '--no-lock']
    subprocess.check_call(command)


def test_wheel(path):
    test_script = os.path.join(resource_root, 'macos-test-wheel.sh')
    subprocess.check_call(['bash', test_script, path])


def build(options):
    if options.extract:
        assert_isdir(options.output_dir, 'Output location')

    assert_isdir(options.build_root, 'Build root')

    provision()

    build_script = os.path.join(resource_root, 'macos-build-wheel.sh')
    environment = os.environ.copy()
    environment['DRAKE_WHEELBUILD_PREFIX'] = options.build_root

    subprocess.check_call(['bash', build_script, options.version],
                          env=environment)

    wheel = find_wheel(os.path.join(options.build_root, 'wheel', 'dist'))

    if options.test:
        test_wheel(wheel)

    if options.extract:
        shutil.copy2(wheel, options.output_dir)


def add_build_arguments(parser):
    parser.add_argument(
        '-r', '--build-root', metavar='DIR', default='/',
        help='root directory for build trees (default: /)')


def add_selection_arguments(parser):
    pass  # macOS can only build one wheel.


def fixup_options(options):
    pass  # Not needed on macOS.
