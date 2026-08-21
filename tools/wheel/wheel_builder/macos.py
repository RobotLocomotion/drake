# This file contains macOS-specific logic used to build the PyPI wheels. See
# build-wheels for the user interface.

import glob
import os
import platform
import shutil
import subprocess
import tempfile

from .common import (
    build_root,
    create_snopt_tgz,
    die,
    find_tests,
    resource_root,
    test_root,
    wheel_name,
    wheelhouse,
)
from .macos_types import PythonTarget

# This is the complete set of defined targets (i.e. potential wheels). By
# default, all targets are built, but the user may down-select from this set.
# On macOS (unlike Linux), this is just the set of Python versions targeted.
#
# These should be kept in sync with
# `setup/macos/source_distribution/Brewfile-developer`.
python_targets = (
    # NOTE: adding or removing a python version?  Please also check the
    # following locations for updates:
    # * the artifact tallies in doc/_pages/release_playbook.md (search
    #   `Attach binaries`);
    # * the set of Python versions for which lockfiles are
    #   generated in tools/workspace/python/venv_upgrade;
    # * the set of Python versions for which installation via `pip` is
    #   supported in doc/_pages/installation.md (search `when installing via
    #   ``pip```);
    # * the Python versions supported by MOSEK, in tools/wheel/setup.py. If
    #   there is any Python version supported by Drake, but not MOSEK, a note
    #   should be added to the aforementioned installation documentation.
    PythonTarget(3, 13),
    PythonTarget(3, 14),
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
        wheel_platform="*",
    )

    candidates = glob.glob(os.path.join(path, pattern))

    if len(candidates) != 1:
        die(
            "Build was expected to produce exactly 1 wheel, "
            f"but {len(candidates)} were found!"
        )

    return candidates[0]


def _assert_isdir(path, name):
    """
    Asserts that `path` is a directory. Shows an error otherwise, where `name`
    describes what path is being asserted.
    """
    if not os.path.isdir(path):
        die(f"{name} '{path}' is not a valid directory")


def _test_wheel(wheel, python_target, env):
    """
    Runs the test script on `wheel`.
    """
    setup_script = os.path.join(
        resource_root, "macos", "provision-test-python.sh"
    )
    subprocess.check_call(
        ["bash", setup_script, python_target.version], env=env
    )

    test_python_venv = os.path.join(test_root, "python")
    os.symlink(
        os.path.join(test_root, f"python{python_target.version}"),
        test_python_venv,
    )

    # Install the wheel.
    install_script = os.path.join(resource_root, "test", "install-wheel.sh")
    subprocess.check_call(["bash", install_script, wheel], env=env)

    # Run individual tests.
    test_script = os.path.join(resource_root, "test", "test-wheel.sh")
    for test in find_tests():
        print(f"-- Executing test {test}")
        subprocess.check_call(["bash", test_script, test, wheel], env=env)
        print(f"-- Executing test {test} - PASSED")

    os.unlink(test_python_venv)


def build(options):
    """
    Builds wheel(s) with the provided options.
    """
    if options.extract:
        _assert_isdir(options.output_dir, "Output location")

    # Collect set of wheels to be built.
    targets_to_build = []
    for t in python_targets:
        if t.tag in options.python_versions:
            targets_to_build.append(t)

    # Check if there is anything to do.
    if not len(targets_to_build):
        die(
            "Nothing to do! (Python version selection "
            "resulted in an empty set of wheels)"
        )

    # Sanitize the build/test environment.
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("RUNFILES_MANIFEST_FILE")

    # Xcode updates may change the default -mmacosx-version-min when not
    # specified, which can result in compiling for the wrong deployment target
    # and ending up with a wheel that doesn't run our supported platforms.
    # To guard against that, we'll set this environment variable that controls:
    # - The majority of the apple and python tooling behind the scenes.
    # - The value for the bazel argument --macos_minimum_os used in
    #   tools/wheel/macos/build-wheel.sh.
    # We always target the macOS that is building the wheel, so to define the
    # deployment target we use the macOS product version X.Y.Z and set X.0.
    deployment_target = f"{platform.mac_ver()[0].split('.')[0]}.0"
    environment["MACOSX_DEPLOYMENT_TARGET"] = deployment_target

    # gfortran hard-codes the path to the SDK with which it was built, which
    # may not match the SDK actually on the machine. This can result in the
    # error "ld: library not found for -lm", and can be fixed/overridden by
    # setting SDKROOT to the appropriate path.
    sdk_path = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True)
    environment["SDKROOT"] = sdk_path.strip()

    # Inject the build version into the environment.
    environment["DRAKE_VERSION"] = options.version

    # Create some scratch space for temporary storage.
    scratch_root = os.path.expanduser("~/.drake-wheel-build")
    os.makedirs(scratch_root, exist_ok=True)
    scratch_dir = tempfile.TemporaryDirectory(
        dir=scratch_root, prefix="scratch-"
    )

    # Create the snopt source archive (and pass along as an environment var).
    snopt_tgz = os.path.join(scratch_dir.name, "snopt.tar.gz")
    environment["SNOPT_PATH"] = snopt_tgz
    create_snopt_tgz(snopt_path=options.snopt_path, output=snopt_tgz)

    # Build the wheel(s).
    for python_target in targets_to_build:
        build_script = os.path.join(resource_root, "macos", "build-wheel.sh")
        build_command = ["bash", build_script]
        build_command.append(options.version)
        build_command.append(python_target.version)

        subprocess.check_call(build_command, env=environment)

        # Find the built wheel and, if requested, test and/or extract it.
        wheel = _find_wheel(
            path=wheelhouse,
            version=options.version,
            python_target=python_target,
        )

        if options.test:
            _test_wheel(wheel, python_target=python_target, env=environment)

        if options.extract:
            shutil.copy2(wheel, options.output_dir)

        if not options.keep_build:
            shutil.rmtree(os.path.realpath(build_root))
            os.unlink(build_root)

            if options.test:
                shutil.rmtree(os.path.realpath(test_root))
                os.unlink(test_root)


def add_build_arguments(parser):
    """
    Adds arguments that control the build.
    """
    parser.add_argument(
        "-k",
        "--keep-build",
        action="store_true",
        help=(
            "do not delete build/test trees on success "
            "(tree(s) are always retained on failure)"
        ),
    )


def add_selection_arguments(parser):
    """
    Adds arguments that control which wheel(s) to build.
    """
    parser.add_argument(
        "--python",
        dest="python_versions",
        metavar="VERSIONS",
        default=",".join(sorted([t.tag for t in python_targets])),
        help=(
            "python version(s) to build; "
            "separate with ',' (default: %(default)s)"
        ),
    )


def fixup_options(options):
    """
    Validates options and applies any necessary transformations.
    (Converts comma-separated strings to sets.)
    """
    options.python_versions = set(options.python_versions.split(","))
