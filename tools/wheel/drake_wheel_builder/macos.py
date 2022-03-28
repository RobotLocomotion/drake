# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import distutils
import os
import platform
import re
import subprocess

from .common import die, wheel_name

resource_root = None

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def provision():
    packages_path = os.path.join(resource_root, 'image', 'packages-macos')
    command = ['brew', 'bundle', f'--file={packages_path}', '--no-lock']
    subprocess.check_call(command)


def test_wheel(path):
    pass  # TODO


def build(options):
    if not os.path.isdir(options.build_root):
        die(f'Build root \'{options.build_root}\' is not a valid directory')

    provision()

    build_script = os.path.join(resource_root, 'macos-build-wheel.sh')
    environment = os.environ.copy()
    environment['DRAKE_WHEELBUILD_PREFIX'] = options.build_root

    subprocess.check_call(['bash', build_script, options.version],
                          env=environment)

    wheel = wheel_name(
        python_version=''.join(platform.python_version_tuple()[:2]),
        wheel_platform=re.sub('[.-]', '_', distutils.util.get_platform()),
        wheel_version=options.version)
    wheel_path = os.path.join(
        options.build_root, 'wheel', 'dist', wheel)

    if options.test:
        test_wheel(wheel_path)

    # TODO extract (copy) wheel if requested


def add_build_arguments(parser):
    parser.add_argument(
        '-r', '--build-root', metavar='DIR', default='/',
        help='root directory for build trees (default: /)')


def add_selection_arguments(parser):
    pass  # macOS can only build one wheel.


def fixup_options(options):
    pass  # Not needed on macOS.
