# This file contains Linux-specific logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import atexit
from datetime import datetime, timezone
import os
import pathlib
import subprocess
import sys
import tarfile

from .common import (
    create_snopt_tgz,
    die,
    find_tests,
    gripe,
    resource_root,
    strip_tar_metadata,
    wheel_name,
    wheelhouse,
)
from .linux_types import BUILD, TEST, Platform, Role, Target

# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_files_to_remove = []
_images_to_remove = []

tag_base = "pip-drake"

# This is the complete set of defined targets (i.e. potential wheels). By
# default, all targets are built, but the user may down-select from this set.
# The platform alias is used for Docker tag names, and, when combined with the
# Python version, must be unique.
targets = (
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
    Target(
        build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
        test_platform=Platform("ubuntu", "22.04", "jammy"),
        python_version_tuple=(3, 10, 16),
        python_sha="bfb249609990220491a1b92850a07135ed0831e41738cf681d63cf01b2a8fbd1",  # noqa
    ),
    Target(
        build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
        test_platform=Platform("ubuntu", "22.04", "jammy"),
        python_version_tuple=(3, 11, 11),
        python_sha="2a9920c7a0cd236de33644ed980a13cbbc21058bfdc528febb6081575ed73be3",  # noqa
    ),
    Target(
        build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
        test_platform=Platform("ubuntu", "24.04", "noble"),
        python_version_tuple=(3, 12, 8),
        python_sha="c909157bb25ec114e5869124cc2a9c4a4d4c1e957ca4ff553f1edc692101154e",  # noqa
    ),
    Target(
        build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
        # TODO(jwnimmer-tri) Switch testing to 26.04 once it's been released.
        test_platform=Platform("ubuntu", "25.10", "questing"),
        python_version_tuple=(3, 13, 0),
        python_sha="086de5882e3cb310d4dca48457522e2e48018ecd43da9cdf827f6a0759efb07d",  # noqa
    ),
    Target(
        build_platform=Platform("amd64/almalinux", "9", "almalinux9"),
        # TODO(jwnimmer-tri) Switch testing to 26.04 once it's been released.
        test_platform=Platform("ubuntu", "25.10", "questing"),
        python_version_tuple=(3, 14, 0),
        python_sha="2299dae542d395ce3883aca00d3c910307cd68e0b2f7336098c8e7b7eee9f3e9",  # noqa
    ),
)
glibc_versions = {
    "almalinux9": "2_34",
}


def _path_depth(path):
    """
    Return the number of components (i.e. the "depth") of `path`.
    """
    offset = 1 if os.path.isabs(path) else 0  # Strip leading '/'.
    return len(pathlib.Path(path).parts[offset:])


def _docker(*args, stdout=None):
    """
    Runs a docker command.
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


def _add_to_tar(tar, name, parent_path, root_path, exclude=[]):
    """
    Adds files or directories to the specified tar file.
    """
    tar_path = os.path.join(parent_path, name)
    full_path = os.path.join(root_path, parent_path, name)

    if os.path.isdir(full_path):
        for f in sorted(os.listdir(full_path)):
            if f in exclude:
                continue

            _add_to_tar(tar, f, os.path.join(parent_path, name), root_path)
    else:
        tar.add(full_path, tar_path, recursive=False, filter=strip_tar_metadata)


def _create_source_tar(path):
    """
    Creates a tarball of the repository working tree.
    """
    print("[-] Creating source archive", end="", flush=True)
    out = tarfile.open(path, "w")

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


def _tagname(target: Target, role: Role, tag_prefix: str):
    """
    Generates a Docker tag name for a target and tag prefix.
    """
    platform = target.platform(role).alias
    return f"{tag_base}:{tag_prefix}-{platform}-py{target.python_tag}"


def _build_stage(target, args, tag_prefix, stage=None):
    """
    Runs a Docker build and return the build tag.
    """

    # Generate canonical tag from target.
    tag = _tagname(target, BUILD, tag_prefix)

    # Generate extra arguments to specify what stage to build.
    if stage is not None:
        extra = ["--target", stage]
    else:
        extra = []

    # Run the build.
    print("[-] Build", tag, extra + args)
    _docker("build", "--tag", tag, *extra, *args, resource_root)

    return tag


def _target_args(target: Target, role: Role):
    """
    Returns the docker build arguments for the specified platform target.
    """
    platform_name = target.platform(role).name
    platform_version = target.platform(role).version
    python_version = target.python_version

    if role == BUILD and target.python_sha is not None:
        python_args = [
            "--build-arg", f"PYTHON=build:{target.python_version_full}",
            "--build-arg", f"PYTHON_SHA={target.python_sha}",
        ]  # fmt: skip
    else:
        python_args = [
            "--build-arg", f"PYTHON={python_version}",
        ]  # fmt: skip

    return [
        "--build-arg", f"PLATFORM={platform_name}:{platform_version}",
    ] + python_args  # fmt: skip


def _build_image(target, identifier, options):
    """
    Runs the build for a target and (optionally) extract the wheel.
    """
    args = [
        "--build-arg", f"DRAKE_VERSION={options.version}",
        "--build-arg", f"DRAKE_GIT_SHA={_git_sha(resource_root)}",
    ] + _target_args(target, BUILD)  # fmt: skip
    if not options.keep_containers:
        args.append("--force-rm")

    # Build the image.
    if options.tag_stages:
        # Inspect Dockerfile, find stages, and build them.
        dockerfile = os.path.join(resource_root, "Dockerfile")
        for line in open(dockerfile, encoding="utf-8"):
            if line.startswith("FROM"):
                stage = line.strip().split()[-1]
                tag = _build_stage(target, args, tag_prefix=stage, stage=stage)
    else:
        tag = _build_stage(target, args, tag_prefix=identifier)
        _images_to_remove.append(tag)

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


def _test_wheel(target, identifier, options):
    """
    Runs the test script for the wheel matching the specified target.
    """
    glibc = glibc_versions[target.platform(BUILD).alias]
    wheel = wheel_name(
        python_version=target.python_tag,
        wheel_version=options.version,
        wheel_platform=f"manylinux_{glibc}_x86_64",
    )

    test_image = _tagname(target, TEST, f"test-{identifier}")
    test_container = test_image.replace(":", "__")
    if options.tag_stages:
        base_image = _tagname(target, TEST, "test")
    else:
        base_image = test_image
    test_dir = os.path.join(resource_root, "test")

    # Build the test base image.
    _docker("build", "-t", base_image, *_target_args(target, TEST), test_dir)
    if not options.tag_stages:
        _images_to_remove.append(base_image)

    # Install the wheel.
    install_script = "/test/install-wheel.sh"
    _docker(
        "run", "-t", f"--name={test_container}",
        f"-v{test_dir}:/test",
        f"-v{options.output_dir}:{wheelhouse}",
        base_image, install_script, os.path.join(wheelhouse, wheel),
    )  # fmt: skip

    # Tag the container with the wheel installed.
    _docker("commit", test_container, test_image)
    _docker("container", "rm", test_container)
    if options.tag_stages:
        _images_to_remove.append(test_image)

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
        if t.platform(BUILD).name in options.platforms:
            if t.python_tag in options.python_versions:
                targets_to_build.append(t)

    # Check if there is anything to do.
    if not len(targets_to_build):
        die(
            "Nothing to do! (Platform and/or Python version selection "
            "resulted in an empty set of wheels)"
        )

    # Generate a unique identifier for this build.
    salt = os.urandom(8).hex()
    time = datetime.now(timezone.utc).strftime("%Y%m%d%H%M%S")
    identifier = f"{time}-{salt}"

    # Provide the SNOPT source archive as a dependency.
    snopt_tgz = os.path.join(resource_root, "image", "snopt.tar.gz")
    _files_to_remove.append(snopt_tgz)
    create_snopt_tgz(snopt_path=options.snopt_path, output=snopt_tgz)

    # Generate the Drake repository source archive.
    source_tar = os.path.join(resource_root, "image", "drake-src.tar")
    _files_to_remove.append(source_tar)
    _create_source_tar(source_tar)

    # Build the requested wheels.
    for t in targets_to_build:
        _build_image(t, identifier, options)

        if options.test:
            _test_wheel(t, identifier, options)


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
        default=",".join(set([t.platform(BUILD).name for t in targets])),
        help="platform(s) to build; separate with ',' (default: %(default)s)",
    )
    parser.add_argument(
        "--python",
        dest="python_versions",
        metavar="VERSIONS",
        default=",".join(sorted(set([t.python_tag for t in targets]))),
        help=(
            "python version(s) to build; separate with ','"
            " (default: %(default)s)",
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
