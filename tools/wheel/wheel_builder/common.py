# This file contains shared logic used to build the PyPI wheels. See
# //tools/wheel:builder for the user interface.

import argparse
import gzip
import io
import locale
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tarfile

# Location where most of the build will take place. This is a symlink to the
# actual build directory which a) is unique per build, and b) resides in the
# user's home directory, where we can be confident that there is enough disk
# space for a build (since `/tmp` might be on `tmpfs`).
build_root = "/tmp/drake-wheel-build"

# Location where testing of the wheel will take place. On macOS, this is a
# symlink with similar semantics to `build_root`. On Linux, we don't require
# the uniqueness because tests are run in a container, and don't require the
# guarantee that we are not on `tmpfs` because tests do not require nearly as
# much disk space as builds.
test_root = "/tmp/drake-wheel-test"

# Location where the wheel will be produced.
wheel_root = os.path.join(build_root, "drake-wheel")
wheelhouse = os.path.join(wheel_root, "wheelhouse")

# Location of various scripts and other artifacts used to complete the build.
resource_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def gripe(message):
    """
    Prints a message to stderr.
    """
    print(message, file=sys.stderr)


def die(message, result=1):
    """
    Prints a message to stderr and aborts.
    """
    gripe(message)
    sys.exit(result)


def wheel_name(python_version, wheel_version, wheel_platform):
    """
    Determines the complete name of the Drake wheel, given various individual
    bits such as the Drake version, Python version, and Python wheel platform.
    """
    vm = f"cp{python_version}"
    return f"drake-{wheel_version}-{vm}-{vm}-{wheel_platform}.whl"


def _check_version(version):
    """
    Returns True iff the given version string matches PEP 440.
    """
    return (
        re.match(
            r"^([1-9][0-9]*!)?(0|[1-9][0-9]*)"
            r"(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?"
            r"(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?"
            r"([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$",
            version,
        )
        is not None
    )


def strip_tar_metadata(info: tarfile.TarInfo):
    """
    Removes some metadata (owner, timestamp) from a TarInfo.
    """
    info.uid = info.gid = 0
    info.uname = info.gname = "root"
    info.mtime = 0
    info.pax_headers = {}
    return info


def create_snopt_tgz(*, snopt_path, output):
    """
    If `snopt_path` is 'git', then fetches the SNOPT source code from git and
    compresses it onto the given `output` filename (typically 'snopt.tar.gz').
    Otherwise, just copies the `snopt_path` file to `output`.
    """
    if snopt_path != "git":
        shutil.copy(src=snopt_path, dst=output)
        return

    print("[-] Creating SNOPT archive...", flush=True)
    tar_buffer = io.BytesIO()
    tar_writer = tarfile.open(mode="w", fileobj=tar_buffer)

    # Ask Bazel where it keeps its externals.
    command = ["bazel", "info", "output_base"]
    output_base = subprocess.check_output(
        command, cwd=resource_root, stderr=subprocess.DEVNULL, encoding="utf-8"
    ).strip()
    bazel_snopt = os.path.join(
        output_base, "external/+drake_dep_repositories+snopt"
    )

    # Ask Bazel to fetch SNOPT from its default git pin.
    command = [
        "bazel",
        "fetch",
        "@snopt//:snopt_cwrap",
        "--repo_env=SNOPT_PATH=git",
    ]
    subprocess.run(command, check=True, cwd=resource_root)

    # Compress the files into a tar archive. We only want the files from these
    # subdirectories (and not recursively):
    keep_dirs = [
        "interfaces/include",
        "interfaces/src",
        "src",
    ]
    for keep_dir in keep_dirs:
        full_dir = os.path.join(bazel_snopt, keep_dir)
        for name in sorted(os.listdir(full_dir)):
            # Compute full_file as the local path, and tgz_file as the path
            # as it will exist within the archive. Only add files, not dirs.
            full_file = os.path.join(full_dir, name)
            if not os.path.isfile(full_file):
                continue
            tgz_file = os.path.join("snopt", keep_dir, name)
            # Now here's a bit of magic: we need to undo the 'patch' commands
            # that were run by Bazel while fetching, since it will need to run
            # them again inside of the container. If we see filenames 'foo' and
            # 'foo.orig', we should skip 'foo' and add 'foo.orig' as 'foo'.
            if os.path.isfile(full_file + ".orig"):
                continue
            if tgz_file.endswith(".orig"):
                tgz_file = tgz_file[: -len(".orig")]
            # Add the file.
            tar_writer.add(
                full_file, tgz_file, recursive=False, filter=strip_tar_metadata
            )
    tar_writer.close()

    # Write to disk and gzip (as required by Drake's SNOPT_PATH).
    tar_buffer.seek(0)
    tgz_data = gzip.compress(tar_buffer.read(), compresslevel=0, mtime=0)
    pathlib.Path(output).write_bytes(tgz_data)


def find_tests(*test_subdirs):
    """
    Returns a list of tests in the common directory and any subdirectories
    given as additional arguments.
    """
    all_tests = []
    for test_dir in ("", *test_subdirs):
        tests = []

        test_dir_full = os.path.join(resource_root, "test", "tests", test_dir)
        for test in os.listdir(test_dir_full):
            if not os.path.isdir(os.path.join(test_dir_full, test)):
                tests.append(os.path.join("tests", test_dir, test))

        where = f"subdirectory {test_dir!r}" if len(test_dir) else "directory"
        assert len(tests), f"No tests were found in the test {where}!"

        all_tests += sorted(tests)

    return all_tests


def do_main(args, platform):
    """
    Entry point; performs the build using the given CLI arguments, platform,
    and resource root.

    The `platform` must be either a `linux` or `macos` object which provides
    platform-specific implementations of various operations necessary to
    complete the build.
    """

    # Ensure we and any subprocesses are speaking UTF-8.
    locale.setlocale(locale.LC_ALL, "en_US.UTF-8")
    locale_keys = [k for k in os.environ.keys() if k.startswith("LC_")]
    for k in locale_keys:
        os.environ.pop(k)
    os.environ["LC_ALL"] = "en_US.UTF-8"
    os.environ["LANG"] = "en_US.UTF-8"

    # Work around `bazel run` changing the working directory; this is to allow
    # the user to pass in relative paths in a sane manner.
    real_cwd = os.environ.get("BUILD_WORKING_DIRECTORY")
    if real_cwd is not None:
        os.chdir(real_cwd)

    # Set up argument parser.
    parser = argparse.ArgumentParser(
        prog="build-wheels",
        description="Build the Drake PyPI wheel(s).",
    )

    parser.add_argument(
        "version",  # BR
        help="PEP 440 version number with which to label the wheels",
    )

    parser.add_argument(
        "-o",
        "--output-dir",
        metavar="DIR",
        default=os.path.realpath("."),
        help="directory into which to extract wheels (default: %(default)r)",
    )
    parser.add_argument(
        "-n",
        "--no-extract",
        dest="extract",
        action="store_false",
        help="build images but do not extract wheels",
    )
    parser.add_argument(
        "--no-test",
        dest="test",
        action="store_false",
        help="build images but do not run tests",
    )

    parser.add_argument(
        "--snopt-path",
        default="git",
        help="Path to snopt.tgz, otherwise defaults to fetching from git",
    )

    if platform is not None:
        platform.add_build_arguments(parser)
        platform.add_selection_arguments(parser)

    parser.add_argument(
        "--pep440",
        action="store_true",
        help="validate version number without building anything",
    )

    # Parse arguments.
    options = parser.parse_args(args)
    if platform is not None:
        platform.fixup_options(options)

    if not _check_version(options.version):
        die(f"Version '{options.version}' does NOT conform to PEP 440")

    if options.pep440:
        print(f"Version '{options.version}' conforms to PEP 440")
        return

    if options.snopt_path != "git":
        if not os.path.exists(options.snopt_path):
            die(f"The snopt-file path '{options.snopt_path}' does not exist")

    if platform is not None:
        platform.build(options)
    else:
        die(
            "Building wheels is not supported on this platform "
            f"('{sys.platform}')"
        )
    print("wheel_builder: SUCCESS")
