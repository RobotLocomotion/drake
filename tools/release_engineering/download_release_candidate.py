r"""
Downloads the to-be-released binaries, verifies they are all the same version,
and prepares to upload them per the release playbook.

Use bazel to build the tool:

Here's an example of how to obtain the git sha for the release.

  bazel run //tools/release_engineering:download_release_candidate -- \
      --timestamp 20220303 --find-git-sha

Here's an example of how download the release artifacts:

  bazel run //tools/release_engineering:download_release_candidate -- \
      --timestamp 20220303 --version v1.0.0

TODO(jwnimmer-tri) Rename this tool to something more general, like
`release_candidate` (without the "download" part).
"""

import argparse
import os
import shlex
import subprocess
import sys
import tarfile
import tempfile


class UserError(RuntimeError):
    pass


def _run(argv, check=True, shell=False, **kwargs):
    """Runs as per subprocess.run, but with added command logging."""
    if shell:
        assert isinstance(argv, str)
        cmd = argv
    else:
        assert isinstance(argv, list)
        cmd = shlex.join(argv)
    print(f"+ {cmd}", file=sys.stderr)
    return subprocess.run(argv, check=check, shell=shell, **kwargs)


def _get_commit_from_version(*, filename):
    # TODO(jwnimmer-tri) Add git sha into whl files for cross-checking as well.
    tar_file = filename
    print(f"Extract version information from: {tar_file}...")
    with tarfile.open(tar_file, "r") as tar:
        version_member = tar.getmember("drake/share/doc/drake/VERSION.TXT")
        with tar.extractfile(version_member) as f:
            assert f is not None, tar_file
            text = f.read().decode("utf8")
    _, commit = text.split()
    assert len(commit) == 40, repr(commit)
    return commit


def _download_with_sha(*, base_url, filename):
    """Downloads {base_url}/{filename} along with its sha512 sum, checks the
    sha512 sum, and generates a sha256 sum as well.
    """
    _run(["wget", f"{base_url}/{filename}.sha512"])
    _run(["wget", f"{base_url}/{filename}"])
    _run(["sha512sum", "-c", f"{filename}.sha512"])
    _run(f"sha256sum {filename} > {filename}.sha256", shell=True)
    _run(["sha256sum", "-c", f"{filename}.sha256"])


def _download_binaries(*, timestamp, wheels, version):
    """Downloads the binaries as specified, and returns a list of (relative)
    paths.

    The `timestamp` is a string like "YYYYMMDD".
    The `wheels` is a bool (whether to download wheels).
    The `version` is a string like "vM.m.p".
    """
    assert (version is None) == (wheels is False)

    # This is a partial inventory of our binary releases (tgz and wheel only).
    # The apt and docker releases are handled separately.
    binaries = {
        "https://drake-packages.csail.mit.edu/drake/nightly": [
            f"drake-{timestamp}-focal.tar.gz",
            f"drake-{timestamp}-jammy.tar.gz",
            f"drake-{timestamp}-mac.tar.gz",
        ],
    }
    if wheels:
        binaries["https://drake-packages.csail.mit.edu/drake/staging"] = [
            f"drake-{version[1:]}-cp38-cp38-manylinux_2_31_x86_64.whl",
            f"drake-{version[1:]}-cp39-cp39-manylinux_2_31_x86_64.whl",
            f"drake-{version[1:]}-cp310-cp310-manylinux_2_31_x86_64.whl",
            f"drake-{version[1:]}-cp310-cp310-macosx_11_0_x86_64.whl",
        ]

    # Download.
    result = []
    for base_url, flavor_filenames in binaries.items():
        for one_filename in flavor_filenames:
            _download_with_sha(base_url=base_url, filename=one_filename)
            result.append(one_filename)
    return result


def _get_consistent_git_commit_sha(*, filenames):
    """Returns the common git sha within the given list of filenames.
    """
    # TODO(jwnimmer-tri) Add git sha into whl files for cross-checking.
    non_wheel_filenames = [
        x for x in filenames
        if not x.endswith(".whl")
    ]
    # Verify that each archive uses the same version.
    commit_list = [
        _get_commit_from_version(filename=x)
        for x in non_wheel_filenames
    ]
    result = commit_list[0]
    version_errors = []
    for one_filename, commit in zip(non_wheel_filenames, commit_list):
        if commit != result:
            version_errors.append(
                f"For '{one_filename}': Commit '{commit}' is not "
                f"the expected value '{result}'")
    if len(version_errors) > 0:
        raise UserError("\n".join(version_errors))
    return result


def _find_git_sha(*, timestamp):
    """Implements the --find-git-sha command line action.
    """
    with tempfile.TemporaryDirectory(prefix="drake-release-tmp-") as tmp_dir:
        print(f"+ cd {tmp_dir}", file=sys.stderr)
        os.chdir(tmp_dir)
        filenames = _download_binaries(
            timestamp=timestamp, wheels=False, version=None)
        result = _get_consistent_git_commit_sha(filenames=filenames)
        print()
        print(f"The nightly binaries all have the same commit: {result}")


def _download_version(*, timestamp, version):
    """Implements the --version (download) command line action.
    """
    if version[0] != "v":
        raise UserError(f"Bad version format: {version}")
    tmp_dir = f"/tmp/drake-release/{version}"
    if os.path.isdir(tmp_dir):
        raise UserError(
            f"Directory must not already exist: {tmp_dir}\n"
            f"If you are re-running this script, please remove the "
            f"directory.")
    os.makedirs(tmp_dir)
    print(f"+ cd {tmp_dir}", file=sys.stderr)
    os.chdir(tmp_dir)

    filenames = _download_binaries(
        timestamp=timestamp, wheels=True, version=version)
    git_sha = _get_consistent_git_commit_sha(filenames=filenames)
    print(f"The binaries all have the same git commit sha: {git_sha}")

    print()
    print(
        f"Ready! Please continue with the release playbook. The files "
        f"to uploaded are located in the following folder (Ctrl+Click "
        f"in the terminal to open in your file explorer):\n"
        f"\n"
        f"  file://{tmp_dir}\n")


def main():
    parser = argparse.ArgumentParser(
        prog="download_release_candidate", description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "--timestamp", type=str, required=True,
        help="Drake archive timestamp in YYYYMMDD format.")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument(
        "--find-git-sha", action="store_true",
        help="Print the git sha to use for this release")
    action.add_argument(
        "--version", type=str,
        help="Release version in vX.Y.Z format.")
    args = parser.parse_args()

    if args.version is None:
        _find_git_sha(timestamp=args.timestamp)
    else:
        _download_version(timestamp=args.timestamp, version=args.version)


if __name__ == "__main__":
    try:
        main()
    except UserError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
