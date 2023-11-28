# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import glob
import os
import platform
import shutil
import subprocess

from .common import die, wheel_name
from .common import build_root, resource_root, wheelhouse
from .common import test_root, find_tests


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


def _find_host_key_types(host):
    key_types = []

    try:
        command = ['ssh-keygen', '-F', host]
        known_keys = subprocess.check_output(command, text=True)

        for line in known_keys.split('\n'):
            if not line or line.startswith('#'):
                continue

            host, key_type, key_hash = line.strip().split()
            key_types.append(key_type)

    except subprocess.CalledProcessError:
        pass

    return key_types


def _provision():
    """
    Prepares wheel build environment.
    """
    packages_path = os.path.join(resource_root, 'image', 'packages-macos')
    command = ['brew', 'bundle', f'--file={packages_path}', '--no-lock']
    subprocess.check_call(command)

    try:
        os.mkdir(os.path.expanduser('~/.ssh'), mode=0o700)
    except FileExistsError:
        pass

    host_keys = ''
    known_hosts_path = os.path.join(resource_root, 'image', 'known_hosts')
    with open(known_hosts_path) as f:
        for line in f:
            host, key_type, key = line.strip().split()
            if key_type in _find_host_key_types(host):
                continue

            host_keys += line

    if host_keys:
        with open(os.path.expanduser('~/.ssh/known_hosts'), 'a') as f:
            f.write(''.join(host_keys))
            os.fchmod(f.fileno(), 0o600)


def _test_wheel(wheel, env):
    """
    Runs the test script on `wheel`.
    """
    setup_script = os.path.join(resource_root, 'macos',
                                'provision-test-python.sh')
    subprocess.check_call(['bash', setup_script], env=env)

    # Install the wheel.
    install_script = os.path.join(resource_root, 'test', 'install-wheel.sh')
    subprocess.check_call(['bash', install_script, wheel], env=env)

    # Run individual tests.
    test_script = os.path.join(resource_root, 'test', 'test-wheel.sh')
    for test in find_tests():
        print(f'-- Executing test {test}')
        subprocess.check_call(['bash', test_script, test, wheel], env=env)
        print(f'-- Executing test {test} - PASSED')


def build(options):
    """
    Builds wheel(s) with the provided options.
    """
    if options.extract:
        _assert_isdir(options.output_dir, 'Output location')

    if options.provision:
        _provision()

    # Sanitize the build/test environment.
    environment = os.environ.copy()
    environment.pop('PYTHONPATH')
    environment.pop('RUNFILES_MANIFEST_FILE')

    # Xcode updates may change the default -mmacosx-version-min when not
    # specified.  For example, Xcode 14.1 on monterey (macOS 12.x) was using a
    # deployment target of 13.0 (ventura), resulting in a wheel that could not
    # be used on monterey.  This environment variable controls:
    # - The majority of the apple and python tooling behind the scenes.
    # - The tools/wheel/image/dependencies/* CMake projects
    #   (CMAKE_OSX_DEPLOYMENT_TARGET initializes to this environment variable).
    # - The value for the bazel argument --macos_minimum_os used in
    #   tools/wheel/macos/build-wheel.sh.
    # We always target the macOS that is building the wheel, so to define the
    # deployment target we use the macOS product version X.Y.Z and set X.0.
    deployment_target = f'{platform.mac_ver()[0].split(".")[0]}.0'
    environment['MACOSX_DEPLOYMENT_TARGET'] = deployment_target

    # Build the wheel dependencies.
    if options.dependencies:
        build_deps_script = os.path.join(resource_root, 'macos',
                                         'build-dependencies.sh')
        subprocess.check_call(['bash', build_deps_script], env=environment)

    # Build the wheel.
    build_script = os.path.join(resource_root, 'macos', 'build-wheel.sh')
    build_command = ['bash', build_script]
    build_command.append(options.version)

    subprocess.check_call(build_command, env=environment)

    # Find the built wheel and, if requested, test and/or extract it.
    wheel = _find_wheel(path=wheelhouse, version=options.version)

    if options.test:
        _test_wheel(wheel, env=environment)

    if options.extract:
        shutil.copy2(wheel, options.output_dir)

    if not options.keep_build:
        shutil.rmtree('/opt/drake-dependencies')
        shutil.rmtree('/opt/drake')
        shutil.rmtree(build_root)
        if options.test:
            shutil.rmtree(test_root)


def add_build_arguments(parser):
    """
    Adds arguments that control the build.
    """
    parser.add_argument(
        '-k', '--keep-build', action='store_true',
        help='do not delete build/test trees on success '
             '(tree(s) are always retained on failure)')
    parser.add_argument(
        '--no-provision', dest='provision', action='store_false',
        help='skip host provisioning '
             '(requires already-povisioned host)')
    parser.add_argument(
        '--no-dependencies', dest='dependencies', action='store_false',
        help='skip building dependencies '
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
