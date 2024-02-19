# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import glob
import os
import platform
import shutil
import subprocess

from .common import die, wheel_name
from .common import build_root, resource_root, wheel_root, wheelhouse
from .common import test_root, find_tests

from .macos_types import PythonTarget

# This is the complete set of defined targets (i.e. potential wheels). By
# default, all targets are built, but the user may down-select from this set.
# On macOS (unlike Linux), this is just the set of Python versions targeted.
python_targets = (
    PythonTarget(3, 11),
    PythonTarget(3, 12),
)


def _find_wheel(path, version, python_target):
    """
    Returns name of built wheel. Uses `glob` to find it, since trying to
    replicate the logic which determines the macOS platform name is not
    accessible and is very non-trivial to replicate.
    """
    pattern = wheel_name(
        python_version=python_target.tag,
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


def _provision(python_targets):
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
    with open(known_hosts_path, 'rt', encoding='utf-8') as f:
        for line in f:
            host, key_type, key = line.strip().split()
            if key_type in _find_host_key_types(host):
                continue

            host_keys += line

    if host_keys:
        known_hosts_path = os.path.expanduser('~/.ssh/known_hosts')
        with open(known_hosts_path, 'at', encoding='utf-8') as f:
            f.write(''.join(host_keys))
            os.fchmod(f.fileno(), 0o600)

    for t in python_targets:
        subprocess.check_call(['brew', 'install', f'python@{t.version}'])


def _test_wheel(wheel, python_target, env):
    """
    Runs the test script on `wheel`.
    """
    setup_script = os.path.join(resource_root, 'macos',
                                'provision-test-python.sh')
    subprocess.check_call(['bash', setup_script, python_target.version],
                          env=env)

    test_python_venv = os.path.join(test_root, 'python')
    os.symlink(os.path.join(test_root, f'python{python_target.version}'),
               test_python_venv)

    # Install the wheel.
    install_script = os.path.join(resource_root, 'test', 'install-wheel.sh')
    subprocess.check_call(['bash', install_script, wheel], env=env)

    # Run individual tests.
    test_script = os.path.join(resource_root, 'test', 'test-wheel.sh')
    for test in find_tests():
        print(f'-- Executing test {test}')
        subprocess.check_call(['bash', test_script, test, wheel], env=env)
        print(f'-- Executing test {test} - PASSED')

    os.unlink(test_python_venv)


def build(options):
    """
    Builds wheel(s) with the provided options.
    """
    if options.extract:
        _assert_isdir(options.output_dir, 'Output location')

    # Collect set of wheels to be built.
    targets_to_build = []
    for t in python_targets:
        if t.tag in options.python_versions:
            targets_to_build.append(t)

    # Check if there is anything to do.
    if not len(targets_to_build):
        die('Nothing to do! (Python version selection '
            'resulted in an empty set of wheels)')

    # Set up build environment.
    os.makedirs(build_root, exist_ok=True)
    if options.provision:
        _provision(targets_to_build)

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

    # gfortran hard-codes the path to the SDK with which it was built, which
    # may not match the SDK actually on the machine. This can result in the
    # error "ld: library not found for -lm", and can be fixed/overridden by
    # setting SDKROOT to the appropriate path.
    sdk_path = subprocess.check_output(['xcrun', '--show-sdk-path'], text=True)
    environment['SDKROOT'] = sdk_path.strip()

    # Build the wheel dependencies.
    if options.dependencies:
        build_deps_script = os.path.join(resource_root, 'macos',
                                         'build-dependencies.sh')
        subprocess.check_call(['bash', build_deps_script], env=environment)

    # Build the wheel(s).
    for python_target in targets_to_build:
        wheel_versioned_root = os.path.join(
            build_root, f'python{python_target.version}', 'wheel')
        os.symlink(wheel_versioned_root, wheel_root)

        build_script = os.path.join(resource_root, 'macos', 'build-wheel.sh')
        build_command = ['bash', build_script]
        build_command.append(options.version)
        build_command.append(python_target.version)

        subprocess.check_call(build_command, env=environment)

        # Find the built wheel and, if requested, test and/or extract it.
        wheel = _find_wheel(path=wheelhouse, version=options.version,
                            python_target=python_target)

        if options.test:
            _test_wheel(wheel, python_target=python_target, env=environment)

        if options.extract:
            shutil.copy2(wheel, options.output_dir)

        os.unlink(wheel_root)

    if not options.keep_build:
        shutil.rmtree('/opt/drake-dependencies')
        shutil.rmtree('/opt/drake-dist')
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
    """
    parser.add_argument(
        '--python', dest='python_versions', metavar='VERSIONS',
        default=','.join(sorted([t.tag for t in python_targets])),
        help='python version(s) to build; separate with \',\''
             ' (default: %(default)s)')


def fixup_options(options):
    """
    Validates options and applies any necessary transformations.
    (Converts comma-separated strings to sets.)
    """
    options.python_versions = set(options.python_versions.split(','))
