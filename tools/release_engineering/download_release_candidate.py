r"""
Downloads the relevant archives, verifies they are all the same version, and
prepares to upload them per the release playbook.

Use bazel to build and use the tool:

  bazel build //tools/release_engineering:download_release_candidate   # build
  bazel-bin/tools/release_engineering/download_release_candidate       # run

Here's an example of how to download the archives:

  bazel build //tools/release_engineering:download_release_candidate
  bazel-bin/tools/release_engineering/download_release_candidate \
    --version v0.27.0 --timestamp 20210216
"""

import argparse
import os
import shlex
import subprocess
import sys
import tarfile


class UserError(RuntimeError):
    pass


def run(argv, check=True, shell=False, **kwargs):
    if shell:
        assert isinstance(argv, str)
        cmd = argv
    else:
        assert isinstance(argv, list)
        cmd = shlex_join(argv)
    print(f"+ {cmd}", file=sys.stderr)
    return subprocess.run(argv, check=check, shell=shell, **kwargs)


def shlex_join(argv):
    # TODO(eric.cousineau): Replace this with `shlex.join` when we exclusively
    # use Python>=3.8.
    return " ".join(map(shlex.quote, argv))


def get_commit_from_version(tar_file, timestamp):
    print(f"Extract version information from: {tar_file}...")
    with tarfile.open(tar_file, "r") as tar:
        version_member = tar.getmember("drake/share/doc/drake/VERSION.TXT")
        with tar.extractfile(version_member) as f:
            assert f is not None, tar_file
            text = f.read().decode("utf8")
    upload_timestamp, commit = text.split()
    assert upload_timestamp.startswith(timestamp), (
        tar_file, upload_timestamp, commit, timestamp,
    )
    return commit


def main():
    parser = argparse.ArgumentParser(
        prog="download_release_candidate", description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "--version", type=str, required=True,
        help="Release version in vX.Y.Z format.")
    parser.add_argument(
        "--timestamp", type=str, required=True,
        help="Drake archive timestamp in YYYYMMDD format.")
    args = parser.parse_args()

    version = args.version
    timestamp = args.timestamp

    release_dir = f"/tmp/drake-release/{version}"
    if os.path.isdir(release_dir):
        raise UserError(
            f"Directory must not already exist: {release_dir}\n"
            f"If you are re-running this script, please remove the "
            f"directory.")

    os.makedirs(release_dir)
    print(f"+ cd {release_dir}", file=sys.stderr)
    os.chdir(release_dir)

    tar_file_list = [
        f"drake-{timestamp}-bionic.tar.gz",
        f"drake-{timestamp}-focal.tar.gz",
        f"drake-{timestamp}-mac.tar.gz",
    ]

    # Download.
    base_url = "https://drake-packages.csail.mit.edu/drake/nightly"
    for tar_file in tar_file_list:
        run(["wget", f"{base_url}/{tar_file}"])
        run(["wget", f"{base_url}/{tar_file}.sha512"])

    # Verify that each archive uses the same version.
    commit_list = [
        get_commit_from_version(tar_file, timestamp)
        for tar_file in tar_file_list
    ]
    commit_expected = commit_list[0]
    version_errors = []
    for tar_file, commit in zip(tar_file_list, commit_list):
        if commit != commit_expected:
            version_errors.append(
                f"For '{tar_file}': Commit '{commit}' is not "
                f"the expected value '{commit_expected}'")
    if len(version_errors) > 0:
        raise UserError("\n".join(version_errors))
    print(f"All versions have the same commit: {commit_expected}")

    # Checksums.
    # - Verify downloaded sha512.
    run("sha512sum -c *.sha512", shell=True)
    # - Generate sha256.
    for tar_file in tar_file_list:
        run(f"sha256sum {tar_file} > {tar_file}.sha256", shell=True)
    # - Verify.
    run("sha256sum -c *.sha256", shell=True)

    print()
    print(
        f"Ready! Please continue with the release playbook. The files "
        f"to uploaded are located in the following folder (Ctrl+Click "
        f"in the terminal to open in your file explorer):\n"
        f"\n"
        f"  file://{release_dir}\n")


if __name__ == "__main__":
    try:
        main()
    except UserError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
