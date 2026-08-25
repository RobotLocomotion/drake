# This file contains Linux-specific logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import atexit
from datetime import UTC, datetime
import itertools
import os
import platform
import subprocess
import sys
import tarfile

from .common import (
    PythonBinder,
    PythonTarget,
    create_snopt_tgz,
    die,
    edit_wheel_version_for_binder,
    find_tests,
    gripe,
    resource_root,
    strip_tar_metadata,
    wheel_name,
    wheelhouse,
)
from .linux_types import Platform, Target, python_manager_for

# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_files_to_remove = set()
_images_to_remove = set()

tag_base = "pip-drake"

ARCH = platform.machine()

# Supported platforms on which every wheel is tested.
_TEST_PLATFORMS = (
    Platform("amazonlinux", "2023", "AL2023"),
    Platform("ubuntu", "24.04", "noble"),
    Platform("ubuntu", "26.04", "resolute"),
)

# This is the complete set of defined targets (i.e. potential wheels). By
# default, all targets matching the currently running architecture are built,
# but the user may down-select from this set. The platform alias is used for
# Docker tag names, and, when combined with the Python version, must be unique.
targets = {
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
    "x86_64": (
        Target(
            python_binder=PythonBinder.NANOBIND,
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 12, 13),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(
                PythonTarget(3, 12),
                PythonTarget(3, 13),
                PythonTarget(3, 14),
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 12, 13),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(PythonTarget(3, 12),),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 13, 15),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(PythonTarget(3, 13),),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 14, 7),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(PythonTarget(3, 14),),
        ),
    ),
    "aarch64": (
        Target(
            python_binder=PythonBinder.NANOBIND,
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 12, 13),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(
                PythonTarget(3, 12),
                PythonTarget(3, 13),
                PythonTarget(3, 14),
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 12, 13),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(PythonTarget(3, 12),),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 13, 15),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(PythonTarget(3, 13),),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            build_python=PythonTarget(3, 14, 7),
            test_platforms=_TEST_PLATFORMS,
            test_pythons=(PythonTarget(3, 14),),
        ),
    ),
}[ARCH]
glibc_versions = {
    "almalinux9": "2_34",
}


def _docker(*args, stdout=None):
    """
    Runs a Docker command.
    The value of `stdout` is passed through to the subprocess module.
    Blocks until completion and returns a CompletedProcess instance.
    """
    command = ["docker"] + list(args)
    environment = os.environ.copy()
    environment["DOCKER_BUILDKIT"] = "1"
    return subprocess.run(
        command, check=True, stdout=stdout, cwd=resource_root, env=environment
    )


@atexit.register
def _cleanup():
    """
    Removes temporary artifacts on exit.
    """
    for f in _files_to_remove:
        try:
            os.unlink(f)
        except FileNotFoundError:
            gripe(f"Warning: failed to remove '{f}'?")
    if len(_images_to_remove):
        _docker("image", "rm", *_images_to_remove)


def _git_sha(path):
    """
    Determines the git SHA of the repository which contains or is rooted at the
    specified `path`.
    """
    command = ["git", "rev-parse", "HEAD"]
    raw = subprocess.check_output(command, cwd=path)
    return raw.decode(sys.stdout.encoding).strip()


def _git_root(path):
    """
    Determines the canonical repository root of the working tree which includes
    `path`.
    """
    command = ["git", "rev-parse", "--show-toplevel"]
    raw = subprocess.check_output(command, cwd=path)
    return raw.decode(sys.stdout.encoding).rsplit("\n", maxsplit=1)[0]


def _add_to_tar(tar, name, parent_path, root_path, exclude=None):
    """
    Adds files or directories to the specified tar file.
    """
    tar_path = os.path.join(parent_path, name)
    full_path = os.path.join(root_path, parent_path, name)

    if os.path.isdir(full_path):
        for f in sorted(os.listdir(full_path)):
            if exclude is not None and f in exclude:
                continue

            _add_to_tar(tar, f, os.path.join(parent_path, name), root_path)
    else:
        tar.add(full_path, tar_path, recursive=False, filter=strip_tar_metadata)


def _create_source_tar(path):
    """
    Creates a tarball of the repository working tree.
    """
    print("[-] Creating source archive", end="", flush=True)
    out = tarfile.open(path, "w")  # noqa: SIM115

    # Walk the git root and archive almost every file we find.
    repo_dir = _git_root(resource_root)
    for f in sorted(os.listdir(repo_dir)):
        # Exclude build and VCS directories.
        if f == ".git" or f == "user.bazelrc" or f.startswith("bazel-"):
            continue

        # Exclude host-generated setup files.
        if f == "gen":
            continue

        # Never add our output (wheel files) back in as input.
        if f.endswith(".whl"):
            continue

        print(".", end="", flush=True)
        exclude = ["wheel"] if f == "tools" else []
        _add_to_tar(out, f, "", repo_dir, exclude=exclude)

    print(" done")
    out.close()


def _build_tagname(target: Target, tag_prefix: str) -> str:
    """
    Generates a Docker tag name for a build-role target and tag prefix.
    """
    platform = target.build_platform.alias
    python_tag = target.build_python.tag
    python_binder = target.python_binder.value
    return f"{tag_base}:{tag_prefix}-{platform}-py{python_tag}-{python_binder}"


def _test_tagname(test_platform, test_python, python_manager, stage) -> str:
    """
    Generates a Docker tag name from the inputs.
    """
    platform = test_platform.alias
    manager = python_manager.value
    if stage == "base":
        return f"{tag_base}:{stage}-{platform}-{manager}"
    python_tag = test_python.tag
    return f"{tag_base}:{stage}-{platform}-py{python_tag}-{manager}"


def _build_dockerfile_stages(dockerfile_dir, args, tagname_fn):
    """
    Builds each named stage of the Dockerfile found in `dockerfile_dir`,
    in order, tagging each with `tagname_fn(stage)`. Returns the tag of the
    final stage built.
    """
    dockerfile = os.path.join(dockerfile_dir, "Dockerfile")
    tag = None
    with open(dockerfile, encoding="utf-8") as f:
        for line in f:
            if line.startswith("FROM") and " AS " in line:
                stage = line.strip().split()[-1]
                tag = tagname_fn(stage)
                extra = ["--target", stage]
                print("[-] Build", tag, extra + args)
                _docker("build", "-t", tag, *extra, *args, dockerfile_dir)
    assert tag is not None, f"No named stages found in {dockerfile}"
    return tag


def _build_target_args(target: Target):
    """
    Returns the Docker build arguments for the build Dockerfile.
    """
    platform = f"{target.build_platform.name}:{target.build_platform.version}"
    return [
        "--build-arg", f"DRAKE_PYTHON_BINDER={target.python_binder.value}",
        "--build-arg", f"PLATFORM={platform}",
        "--build-arg", f"PYTHON={target.build_python.version_full}",
    ]  # fmt: skip


def _test_target_args(test_platform, test_python, python_manager):
    """
    Returns the Docker build arguments for the test Dockerfile.
    """
    platform = f"{test_platform.name}:{test_platform.version}"
    return [
        "--build-arg", f"PLATFORM={platform}",
        "--build-arg", f"PYTHON={test_python.version_full}",
        "--build-arg", f"PYTHON_MANAGER={python_manager.value}",
    ]  # fmt: skip


def _build_image(target, identifier, version, options):
    """
    Runs the build for a target and (optionally) extracts the wheel.
    """
    drake_is_abi3_wheel = (
        "1" if target.python_binder == PythonBinder.NANOBIND else "0"
    )
    args = [
        "--build-arg", f"DRAKE_VERSION={version}",
        "--build-arg", f"DRAKE_GIT_SHA={_git_sha(resource_root)}",
        "--build-arg", f"DRAKE_IS_ABI3_WHEEL={drake_is_abi3_wheel}",
    ] + _build_target_args(target)  # fmt: skip
    if not options.keep_containers:
        args.append("--force-rm")

    # Build the image.
    if options.tag_stages:
        tag = _build_dockerfile_stages(
            resource_root, args, lambda stage: _build_tagname(target, stage)
        )
    else:
        tag = _build_tagname(target, identifier)
        print("[-] Build", tag, args)
        _docker("build", "--tag", tag, *args, resource_root)
        _images_to_remove.add(tag)

    # Extract the wheel (if requested).
    if options.extract:
        print("[-] Extracting wheel(s) from", tag)
        container_name = f"{tag}.extract".replace(":", ".")
        completed_process = _docker(
            "run", f"--name={container_name}", tag,
            "bash", "-c", f"ls {wheelhouse}/*.whl",
            stdout=subprocess.PIPE,
        )  # fmt: skip
        try:
            wheel_paths = completed_process.stdout.decode("utf-8").splitlines()
            assert len(wheel_paths) > 0
            for container_path in wheel_paths:
                wheel_basename = os.path.basename(container_path)
                _docker(
                    "cp",
                    f"{container_name}:{container_path}",
                    os.path.join(options.output_dir, wheel_basename),
                )
        finally:
            _docker("rm", container_name)


def _test_wheel(target, identifier, version, options):
    """
    Runs the test script for the wheel matching the specified target.
    """
    glibc = glibc_versions[target.build_platform.alias]
    wheel = wheel_name(
        python_binder=target.python_binder,
        python_version=target.build_python.tag,
        wheel_version=version,
        wheel_platform=f"manylinux_{glibc}_{ARCH}",
    )
    test_dir = os.path.join(resource_root, "test")

    for test_platform, test_python in itertools.product(
        target.test_platforms, target.test_pythons
    ):
        python_manager = python_manager_for(test_platform, test_python)
        print(
            f"[-] Testing on {test_platform.alias}"
            f" (Python {test_python.version}) ..."
        )
        args = _test_target_args(test_platform, test_python, python_manager)
        test_image = _test_tagname(
            test_platform, test_python, python_manager, f"test-{identifier}"
        )
        test_container = test_image.replace(":", "__")

        # Build the Python-version agnostic base provisioned image.
        if options.tag_stages:
            provisioned_image = _build_dockerfile_stages(
                test_dir,
                args,
                lambda stage: _test_tagname(
                    test_platform, test_python, python_manager, stage
                ),
            )
        else:
            provisioned_image = test_image
            print("[-] Build", provisioned_image, args)
            _docker(
                "build",
                "-t",
                provisioned_image,
                *args,
                test_dir,
            )
            _images_to_remove.add(provisioned_image)

        # Install the wheel.
        install_command = [
            "/test/install-wheel.sh",
            os.path.join(wheelhouse, wheel),
            python_manager.value,
        ]  # fmt: skip
        _docker(
            "run", "-t", f"--name={test_container}",
            f"-v{test_dir}:/test",
            f"-v{options.output_dir}:{wheelhouse}",
            provisioned_image, *install_command,
        )  # fmt: skip

        # Tag the container with the wheel installed.
        _docker("commit", test_container, test_image)
        _docker("container", "rm", test_container)
        if options.tag_stages:
            _images_to_remove.add(test_image)

        # Run individual tests.
        test_script = "/test/test-wheel.sh"
        for test in find_tests("hermetic"):
            print(f"[-] Executing test {test}")
            _docker(
                "run", "--rm", "-t",
                f"-v{test_dir}:/test",
                f"-v{options.output_dir}:{wheelhouse}",
                test_image,
                test_script, f"/test/{test}", os.path.join(wheelhouse, wheel),
            )  # fmt: skip
            print(f"[-] Executing test {test} - PASSED")


def build(options):
    """
    Builds wheel(s) with the provided options.
    """

    # Collect set of wheels to be built.
    targets_to_build = []
    for t in targets:
        if (
            t.build_platform.name in options.platforms
            and t.build_python.tag in options.python_versions
        ):
            targets_to_build.append(t)

    # Check if there is anything to do.
    if not len(targets_to_build):
        die(
            "Nothing to do! (Platform and/or Python version selection "
            "resulted in an empty set of wheels)"
        )

    # Generate a unique identifier for this build.
    salt = os.urandom(8).hex()
    time = datetime.now(UTC).strftime("%Y%m%d%H%M%S")
    identifier = f"{time}-{salt}"

    # Provide the SNOPT source archive as a dependency.
    snopt_tgz = os.path.join(resource_root, "image", "snopt.tar.gz")
    _files_to_remove.add(snopt_tgz)
    create_snopt_tgz(snopt_path=options.snopt_path, output=snopt_tgz)

    # Generate the Drake repository source archive.
    source_tar = os.path.join(resource_root, "image", "drake-src.tar")
    _files_to_remove.add(source_tar)
    _create_source_tar(source_tar)

    # Build the requested wheels.
    for t in targets_to_build:
        version = edit_wheel_version_for_binder(
            t.python_binder, options.version
        )

        _build_image(t, identifier, version, options)

        if options.test:
            _test_wheel(t, identifier, version, options)


def add_build_arguments(parser):
    """
    Adds arguments that control the build.
    """
    parser.add_argument(
        "-k",
        "--keep-containers",
        action="store_true",
        help="do not delete intermediate containers",
    )
    parser.add_argument(
        "-s",
        "--tag-stages",
        action="store_true",
        help="permanently tag individual stages",
    )


def add_selection_arguments(parser):
    """
    Adds arguments that control which wheel(s) to build.
    """
    parser.add_argument(
        "--platform",
        dest="platforms",
        default=",".join({t.build_platform.name for t in targets}),
        help="platform(s) to build; separate with ',' (default: %(default)s)",
    )
    parser.add_argument(
        "--python",
        dest="python_versions",
        metavar="VERSIONS",
        default=",".join(sorted({t.build_python.tag for t in targets})),
        help=(
            "python version(s) to build; separate with ','"
            " (default: %(default)s)"
        ),
    )


def fixup_options(options):
    """
    Validates options and applies any necessary transformations.
    (Converts comma-separated strings to sets.)
    """
    options.python_versions = set(options.python_versions.split(","))
    options.platforms = set(options.platforms.split(","))

    if options.test and not options.extract:
        die("Testing wheels requires wheels to be extracted")
