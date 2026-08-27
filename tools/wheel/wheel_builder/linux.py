# This file contains Linux-specific logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import atexit
from datetime import UTC, datetime
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
from .linux_types import BUILD, TEST, Platform, PythonManager, Role, Target

# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_files_to_remove = set()
_images_to_remove = set()
_built_test_bases = set()

tag_base = "pip-drake"

ARCH = platform.machine()

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
            python=PythonTarget(3, 12, 13),
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble"),
                Platform("ubuntu", "26.04", "resolute", PythonManager.UV),
                # TODO(jwnimmer-tri) We should test this same abi3 wheel on all
                # newer Python versions (so 3.13, 3.14, etc.).
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            python=PythonTarget(3, 12, 13),
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble"),
                Platform("ubuntu", "26.04", "resolute", PythonManager.UV),
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            python=PythonTarget(3, 13, 15),
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble", PythonManager.UV),
                Platform("ubuntu", "26.04", "resolute", PythonManager.UV),
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            python=PythonTarget(3, 14, 7),
            build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble", PythonManager.UV),
                Platform("ubuntu", "26.04", "resolute"),
            ),
        ),
    ),
    "aarch64": (
        Target(
            python_binder=PythonBinder.NANOBIND,
            python=PythonTarget(3, 12, 13),
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble"),
                Platform("ubuntu", "26.04", "resolute", PythonManager.UV),
                # TODO(jwnimmer-tri) We should test this same abi3 wheel on all
                # newer Python versions (so 3.13, 3.14, etc.).
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            python=PythonTarget(3, 12, 13),
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble"),
                Platform("ubuntu", "26.04", "resolute", PythonManager.UV),
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            python=PythonTarget(3, 13, 15),
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble", PythonManager.UV),
                Platform("ubuntu", "26.04", "resolute", PythonManager.UV),
            ),
        ),
        Target(
            python_binder=PythonBinder.PYBIND11,
            python=PythonTarget(3, 14, 7),
            build_platform=Platform("arm64v8/almalinux", "9", "almalinux9"),
            test_platforms=(
                Platform("amazonlinux", "2023", "AL2023"),
                Platform("ubuntu", "24.04", "noble", PythonManager.UV),
                Platform("ubuntu", "26.04", "resolute"),
            ),
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
    platform = target.platform(BUILD)
    python_tag = target.python.tag
    python_binder = target.python_binder.value
    python_details = f"py{python_tag}-{python_binder}"
    return f"{tag_base}:{tag_prefix}-{platform.alias}-{python_details}"


def _test_tagname(target: Target, test_index: int, tag_prefix: str) -> str:
    """
    Generates a Docker tag name for a test-role target, test_index, and tag
    prefix.
    """
    platform = target.platform(TEST, test_index)
    manager = platform.python_manager.value
    if tag_prefix == "base":
        return f"{tag_base}:{tag_prefix}-{platform.alias}-{manager}"
    python_tag = target.python.tag
    return f"{tag_base}:{tag_prefix}-{platform.alias}-py{python_tag}-{manager}"


def _build_stage(
    tag: str, args: list[str], context_dir: str, stage: str | None = None
) -> None:
    """
    Runs a Docker build and tags the image with the given tag.
    """

    # Generate extra arguments to specify what stage to build.
    extra = ["--target", stage] if stage is not None else []

    # Run the build.
    print("[-] Build", tag, extra + args)
    _docker("build", "-t", tag, *extra, *args, context_dir)


def _target_args(target: Target, role: Role, test_index: int | None = None):
    """
    Returns the Docker build arguments for the specified platform target.
    Iff the role is the TEST role, then the test_index must be provided.
    """
    platform = target.platform(role, test_index)

    if role == BUILD:
        python_args = [
            "--build-arg", f"PYTHON={target.python.version_full}",
            "--build-arg", f"DRAKE_PYTHON_BINDER={target.python_binder.value}",
        ]  # fmt: skip
    else:
        python_args = [
            "--build-arg", f"PYTHON={target.python.version}",
            "--build-arg", f"PYTHON_MANAGER={platform.python_manager.value}",
        ]  # fmt: skip

    return [
        "--build-arg", f"PLATFORM={platform.name}:{platform.version}",
    ] + python_args  # fmt: skip


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
    ] + _target_args(target, BUILD)  # fmt: skip
    if not options.keep_containers:
        args.append("--force-rm")

    # Build the image.
    if options.tag_stages:
        # Inspect Dockerfile, find stages, and build them.
        dockerfile = os.path.join(resource_root, "Dockerfile")
        tag = None
        with open(dockerfile, encoding="utf-8") as f:
            for line in f:
                if line.startswith("FROM"):
                    stage = line.strip().split()[-1]
                    tag = _build_tagname(target, stage)
                    _build_stage(tag, args, resource_root, stage)
        assert tag is not None, f"No named stages found in {dockerfile}"
    else:
        tag = _build_tagname(target, identifier)
        _build_stage(tag, args, resource_root)
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
    glibc = glibc_versions[target.platform(BUILD).alias]
    wheel = wheel_name(
        python_binder=target.python_binder,
        python_version=target.python.tag,
        wheel_version=version,
        wheel_platform=f"manylinux_{glibc}_{ARCH}",
    )
    test_dir = os.path.join(resource_root, "test")

    for test_index, test_platform in enumerate(target.test_platforms):
        print(f"[-] Testing on {test_platform.alias} ...")
        args = _target_args(target, TEST, test_index)
        test_image = _test_tagname(target, test_index, f"test-{identifier}")
        test_container = test_image.replace(":", "__")
        if options.tag_stages:
            # Build the base image, shared across multiple targets.
            base_image = _test_tagname(target, test_index, "base")
            if base_image not in _built_test_bases:
                _build_stage(base_image, args, test_dir, "base")
                _built_test_bases.add(base_image)
                _images_to_remove.add(base_image)

            provisioned_image = _test_tagname(target, test_index, "python")
            _build_stage(provisioned_image, args, test_dir, "python")
        else:
            provisioned_image = test_image
            _build_stage(provisioned_image, args, test_dir)
        _images_to_remove.add(provisioned_image)

        # Install the wheel.
        install_command = [
            "/test/install-wheel.sh",
            os.path.join(wheelhouse, wheel),
            test_platform.python_manager.value,
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
            t.platform(BUILD).name in options.platforms
            and t.python.tag in options.python_versions
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
        default=",".join({t.platform(BUILD).name for t in targets}),
        help="platform(s) to build; separate with ',' (default: %(default)s)",
    )
    parser.add_argument(
        "--python",
        dest="python_versions",
        metavar="VERSIONS",
        default=",".join(sorted({t.python.tag for t in targets})),
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
