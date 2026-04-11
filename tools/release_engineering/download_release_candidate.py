r"""
Downloads the to-be-released binaries, verifies they are all the same version,
and prepares to upload them per the release playbook.

When running this tool, the current Drake checkout must have been merged up
to the latest master (or at least, newer than the intended release git sha).

This program is supported only on Ubuntu (not macOS).

Use bazel to build the tool.

Here's an example of how download the release artifacts:

  bazel run //tools/release_engineering:download_release_candidate -- \
      --version vX.Y.Z
"""

# TODO(jwnimmer-tri) Rename this tool to something more general, like
# `release_candidate` (without the "download" part).

import argparse
from io import StringIO
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tarfile
import tempfile
import zipfile


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
    is_tar_file = filename.endswith(".tar.gz")
    is_deb_file = filename.endswith(".deb")
    is_whl_file = filename.endswith(".whl")
    assert is_tar_file or is_deb_file or is_whl_file, (
        f"{filename} did not end in .tar.gz, .deb, or .whl"
    )

    print(f"Extract version information from: {filename}...")
    if is_tar_file:
        with tarfile.open(filename, "r") as tar:
            version_member = tar.getmember("drake/share/doc/drake/VERSION.TXT")
            with tar.extractfile(version_member) as f:
                assert f is not None, filename
                version_txt_contents = f.read().decode("utf8")
    elif is_deb_file:
        with tempfile.TemporaryDirectory(prefix="drake-release-tmp-") as td:
            # Extract the .deb to a temporary directory to inspect VERSION.TXT.
            _run(["dpkg-deb", "-x", filename, td])
            version_txt_path = (
                Path(td)
                / "opt"
                / "drake"
                / "share"
                / "doc"
                / "drake"
                / "VERSION.TXT"
            )
            with open(version_txt_path) as f:
                version_txt_contents = f.read()
    else:
        assert is_whl_file
        with zipfile.ZipFile(filename, "r") as whl:
            version_txt_contents = whl.read(
                "pydrake/doc/drake/VERSION.TXT"
            ).decode("utf-8")

    _, commit = version_txt_contents.split()
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


def _check_urls(*, urls):
    """Check all urls exist.  Fail if any do not exist with an error message
    indicating which URL(s) cannot be downloaded."""
    assert len(urls) > 0
    missing_urls = []  # list[tuple[str, str]] (url, error message)
    for u in urls:
        print(f"+ Verify URL: {u}", file=sys.stderr)
        proc = subprocess.run(["wget", "--spider", u], capture_output=True)
        if proc.returncode != 0:
            missing_urls.append((u, proc.stderr.decode("utf-8")))

    if missing_urls:
        error_message = StringIO()
        error_message.write(
            f"ERROR: found {len(missing_urls)} missing URL(s).\n"
        )
        for url, message in missing_urls:
            error_message.write(f"[X] {url}:\n    {message}\n")
        raise UserError(error_message.getvalue())


def _download_binaries(*, version):
    """Downloads the binaries as specified, and returns a list of (relative)
    paths.

    The `version` is a string like "vM.m.p", or None.
    """
    # This is a partial inventory of our binary releases (tgz and wheel only).
    # The apt and Docker releases are handled separately.
    binaries = {
        "https://drake-packages.csail.mit.edu/drake/staging": [
            # Wheel filenames.
            f"drake-{version[1:]}-cp312-cp312-manylinux_2_34_x86_64.whl",
            f"drake-{version[1:]}-cp313-cp313-manylinux_2_34_x86_64.whl",
            f"drake-{version[1:]}-cp314-cp314-manylinux_2_34_x86_64.whl",
            f"drake-{version[1:]}-cp313-cp313-macosx_15_0_arm64.whl",
            f"drake-{version[1:]}-cp314-cp314-macosx_15_0_arm64.whl",
            # Deb filenames.
            f"drake-dev_{version[1:]}-1_amd64-noble.deb",
            # Tarball filenames.
            f"drake-{version[1:]}-noble.tar.gz",
            f"drake-{version[1:]}-mac-arm64.tar.gz",
        ],
    }

    # Build a list of flat URLs and a list of (base_url, filename) pairs.
    download_urls = []  # list[str]
    base_url_filename_pairs = []  # list[tuple[str, str]]: (base_url, filename)
    for base_url, flavor_filenames in binaries.items():
        for one_filename in flavor_filenames:
            download_urls.append(f"{base_url}/{one_filename}")
            base_url_filename_pairs.append((base_url, one_filename))

    # Make sure all can be downloaded (fail-fast).
    _check_urls(urls=download_urls)

    # Download.
    result = []
    for base_url, filename in base_url_filename_pairs:
        _download_with_sha(base_url=base_url, filename=filename)
        result.append(filename)
    return result


def _get_consistent_git_commit_sha(*, filenames):
    """Returns the common git sha within the given list of filenames."""
    # Verify that each archive uses the same version.
    commit_list = [_get_commit_from_version(filename=x) for x in filenames]
    result = commit_list[0]
    version_errors = []
    for one_filename, commit in zip(filenames, commit_list):
        if commit != result:
            version_errors.append(
                f"For '{one_filename}': Commit '{commit}' is not "
                f"the expected value '{result}'"
            )
    if len(version_errors) > 0:
        raise UserError("\n".join(version_errors))
    return result


def _check_deb_versions(*, filenames, version):
    """Check every `.deb` in filenames has the correct version, fail if not."""
    deb_filenames = [f for f in filenames if f.endswith(".deb")]
    assert len(deb_filenames) > 0, filenames
    deb_versions = []  # list[tuple[str, str]]: (filename, extracted version)
    for deb in deb_filenames:
        proc = subprocess.run(
            ["dpkg-deb", "-f", deb, "Version"], capture_output=True, check=True
        )
        deb_version = proc.stdout.decode("utf-8").strip()
        deb_versions.append((deb, deb_version))
    version_errors = []
    for deb, deb_version in deb_versions:
        # The version looks like "vM.m.p" and deb_version looks like "M.m.p-1".
        if deb_version != f"{version[1:]}-1":
            version_errors.append(
                f"For '{deb}': Version '{deb_version}' is not "
                f"the expected value '{version}'"
            )
    if len(version_errors) > 0:
        raise UserError("\n".join(version_errors))


def _create_tar_gz(*, git_sha, tmp_dir: str, version: str):
    """Creates a Drake tar.gz source archive of the given `git_sha`, storing it
    in the given `tmp_dir`.
    """
    assert len(git_sha) == 40
    assert version[0] == "v"

    # Create the tar.gz using this program's git clone (but at the release sha,
    # not the current revision).
    output_name = f"drake-{version[1:]}-src.tar.gz"
    output_path = f"{tmp_dir}/{output_name}"
    drake_root = Path(__file__).resolve().parent.parent.parent
    assert (drake_root / "MODULE.bazel").exists()
    subprocess.check_call(
        [
            "git",
            "archive",
            "--format=tar.gz",
            f"--output={output_path}",
            f"--prefix=drake-{version[1:]}/",
            git_sha,
        ],
        cwd=drake_root,
    )

    # Write checksum files.
    _run(
        f"sha256sum {output_name} > {output_name}.sha256",
        shell=True,
        cwd=tmp_dir,
    )
    _run(
        f"sha512sum {output_name} > {output_name}.sha512",
        shell=True,
        cwd=tmp_dir,
    )


def _download_version(*, version):
    """Implements the --version (download) command line action."""
    if version[0] != "v":
        raise UserError(f"Bad version format: {version}")
    tmp_dir = f"/tmp/drake-release/{version}"
    if os.path.isdir(tmp_dir):
        raise UserError(
            f"Directory must not already exist: {tmp_dir}\n"
            f"If you are re-running this script, please remove the "
            f"directory."
        )
    os.makedirs(tmp_dir)
    print(f"+ cd {tmp_dir}", file=sys.stderr)
    os.chdir(tmp_dir)

    filenames = _download_binaries(version=version)
    git_sha = _get_consistent_git_commit_sha(filenames=filenames)
    print(f"The binaries all have the same git commit sha: {git_sha}")

    _check_deb_versions(filenames=filenames, version=version)
    print("The debian binaries all have the same version.")

    _create_tar_gz(
        git_sha=git_sha,
        tmp_dir=tmp_dir,
        version=version,
    )
    print("The source archive was created successfully.")

    print()
    print(
        f"Ready! Please continue with the release playbook. The files "
        f"to uploaded are located in the following folder:\n"
        f"\n"
        f"  file://{tmp_dir}\n"
    )


def main():
    parser = argparse.ArgumentParser(
        prog="download_release_candidate",
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--version",
        type=str,
        required=True,
        help="Download artifacts for the vX.Y.Z release.",
    )
    args = parser.parse_args()

    _download_version(version=args.version)


if __name__ == "__main__":
    try:
        main()
    except UserError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
