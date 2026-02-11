"""
Uploads Drake release artifacts.

This is intended for use by Drake maintainers (only).
This program is only supported on Ubuntu Noble 24.04.
"""

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
from typing import Any, Dict, List, Optional
import urllib.request

import boto3
import github3
from github3.repos.release import Asset, Release
from github3.repos.repo import Repository
from github3.repos.tag import RepoTag

_GITHUB_REPO_OWNER = "RobotLocomotion"
_GITHUB_REPO_NAME = "drake"
_GITHUB_REPO_URI = (
    f"https://github.com/{_GITHUB_REPO_OWNER}/{_GITHUB_REPO_NAME}"
)

_ARCHIVE_HASHES = {"sha256", "sha512"}

_AWS_BUCKET = "drake-packages"
_AWS_PREFIX = "drake/release"


class _State:
    def __init__(self, options: Dict[str, Any], release: Release):
        self._options = options
        self._release = release
        self._scratch = tempfile.TemporaryDirectory()
        self._s3 = boto3.client("s3")

        self.source_version = options.source_version

    def _begin(self, action: str, src: str, dst: str = None) -> None:
        """
        Report the start of an action.
        """
        if dst is not None:
            print(f"{action} {src!r} to {dst!r} ...", flush=True, end="")
        else:
            print(f"{action} {src!r} ...", flush=True, end="")

    def _done(self) -> None:
        """
        Report the completion of an action.
        """
        print(" done")

    def _upload_file_github(self, name: str, local_path: str) -> None:
        """
        Uploads the file at 'local_path' to GitHub under the given 'name'.

        If --dry-run was given, rather than actually uploading files to GitHub,
        prints what would be done.
        """
        # Hard code the MIME types, because the mimetype package fails for
        # both file types that we need.
        if local_path.endswith(".tar.gz"):
            content_type = "application/gzip"
        elif local_path.endswith(".sha256") or local_path.endswith(".sha512"):
            content_type = "text/plain"
        else:
            # If we got here, either something is catastrophically wrong, or
            # the script needs to be updated to consider a new case.
            raise RuntimeError(
                "Cannot determine MIME type for file"
                f" {local_path!r} with unexpected extension"
                " (should be one of: .tar.gz, .sha256,"
                " .sha512)."
            )

        if self._options.dry_run:
            print(
                f"push {name!r} ({content_type}) to {self._release.html_url!r}"
            )
        else:
            self._begin("pushing", name, self._release.html_url)
            with open(local_path, "rb") as f:
                self._release.upload_asset(content_type, name, f)
            self._done()

    def _compute_hash(self, path: str, algorithm: str) -> str:
        """
        Compute and return the specified hash for the specified file.
        """
        with open(path, "rb") as f:
            return hashlib.file_digest(f, algorithm).hexdigest()

    def _write_hashfile(
        self, name: str, algorithm: str, local_path: str, hashfile_path: str
    ) -> str:
        """
        Writes the hash of the specified file using the specified algorithm
        to the specified path. Returns the computed hash.
        """
        self._begin(f"computing {algorithm} for", name)

        digest = self._compute_hash(local_path, algorithm)
        with open(hashfile_path, "wt") as f:
            f.write(f"{digest} {name}\n")

        self._done()
        return digest

    def _download_archive_github(self, name: str) -> str:
        """
        Downloads the source archive from the GitHub release. Returns the full
        path to the downloaded file.
        """
        self._begin("downloading", name)

        local_path = os.path.join(self._scratch.name, name)
        archive_url = (
            f"{_GITHUB_REPO_URI}/archive/refs/tags/"
            f"v{self._options.source_version}.tar.gz"
        )
        assert urllib.request.urlretrieve(archive_url, local_path)

        self._done()
        return local_path

    def get_artifacts(self) -> List[Asset]:
        """
        Get artifacts associated with a GitHub release.
        """
        return list(self._release.assets())

    def push_artifact(
        self,
        artifact: Asset,
        bucket: str,
        path: str,
    ) -> None:
        """
        Pushes the specified artifact to S3 under the given 'bucket'/'path'.

        If --dry-run was given, rather than actually pushing files to S3,
        prints what would be done.
        """
        dest = f"s3://{bucket}/{path}"
        if self._options.dry_run:
            print(
                f"push {artifact.name!r} ({artifact.content_type}) to {dest!r}"
            )
        else:
            self._begin("downloading", artifact.name)
            local_path = os.path.join(self._scratch.name, artifact.name)
            assert artifact.download(path=local_path) is not None
            self._done()

            self._begin("pushing", artifact.name, dest)
            self._s3.upload_file(local_path, bucket, path)
            self._done()

    def push_archive(self, name: str) -> None:
        """
        Pushes the release source archive to GitHub, along with computed
        hashes.

        If --dry-run was given, rather than actually pushing files, prints what
        would be done.
        """
        local_path = self._download_archive_github(name)
        self._upload_file_github(name, local_path)

        # Calculate hashes and write hash files
        for algorithm in _ARCHIVE_HASHES:
            hashfile_name = f"{name}.{algorithm}"
            hashfile_path = os.path.join(self._scratch.name, hashfile_name)

            digest = self._write_hashfile(
                name, algorithm, local_path, hashfile_path
            )
            print(f"{name!r} {algorithm}: {digest}")
            self._upload_file_github(hashfile_name, hashfile_path)


def _fatal(msg: str, result: int = 1) -> None:
    width = shutil.get_terminal_size().columns
    for line in msg.split("\n"):
        print(textwrap.fill(line, width), file=sys.stderr)
    sys.exit(result)


def _assert_tty() -> None:
    try:
        subprocess.check_call(["tty", "-s"])
    except subprocess.CalledProcessError:
        _fatal(
            "ERROR: tty was NOT detected. This script may need"
            " various login credentials to be entered interactively."
        )
        sys.exit(1)


def _test_non_empty(path) -> bool:
    """
    Tests if the specified path exists and is a non-empty file.
    """
    if path is None:
        return False

    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return False

    return os.stat(path).st_size > 0


def _check_version(version) -> bool:
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


def _find_tag(repo: Repository, tag: str) -> Optional[RepoTag]:
    """
    Finds the tag <tag> in the repository <repo>.
    """
    for t in repo.tags():
        if t.name == tag:
            return t
    return None


def _push_source(state: _State) -> None:
    """
    Downloads the source .tar artifact and pushes it to GitHub.
    """
    version = state.source_version
    dest_name = f"drake-{version}-src.tar.gz"
    state.push_archive(dest_name)


def _push_s3(state: _State) -> None:
    """
    Downloads GitHub release artifacts and pushes them to S3.
    """
    for artifact in state.get_artifacts():
        dest_path = f"{_AWS_PREFIX}/{artifact.name}"
        state.push_artifact(artifact, _AWS_BUCKET, dest_path)


def main(args: List[str]) -> None:
    parser = argparse.ArgumentParser(prog="push_release", description=__doc__)
    parser.add_argument(
        "--source",
        dest="push_source",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="Mirror source .tar archive to GitHub.",
    )
    parser.add_argument(
        "--s3",
        dest="push_s3",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="Mirror artifacts to S3.",
    )
    parser.add_argument(
        "-n",
        "--dry-run",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="Print what would be done without actually pushing anything.",
    )
    parser.add_argument(
        "--token",
        default="~/.config/readonly_github_api_token.txt",
        help="Use the GitHub an API token read from this path, if it exists."
        " Otherwise, anonymous access will be used, which may run into"
        " rate limitations. (default: %(default)s)",
    )
    parser.add_argument(
        "source_version",
        help="Version tag (x.y.z) of the release to be pushed.",
    )
    options = parser.parse_args(args)

    # Validate version arguments
    if not _check_version(options.source_version):
        _fatal(
            f"ERROR: source_version {options.source_version!r}"
            " is not a valid version number."
        )

    # Ensure execution environment is suitable.
    _assert_tty()
    if options.push_s3:
        if not _test_non_empty("~/.aws/credentials"):
            _fatal(
                "ERROR: AWS credentials were not found."
                "\n\n"
                "The --s3 option requires the ability to push files to S3,"
                " which requires authentication credentials to be provided."
                "\n\n"
                "Fix this by running `aws configure`."
            )

    # Get GitHub repository, release tag, and release object.
    if _test_non_empty(options.token):
        with open(os.path.expanduser(options.token), "r") as f:
            token = f.read().strip()
        gh = github3.login(token=token)
    else:
        gh = github3.GitHub()
    repo = gh.repository(_GITHUB_REPO_OWNER, _GITHUB_REPO_NAME)
    release_tag = _find_tag(repo, f"v{options.source_version}")

    if release_tag is None:
        _fatal(f"ERROR: GitHub tag v{options.source_version} does NOT exist.")

    release = repo.release_from_tag(release_tag)

    if release is None:
        _fatal(f"ERROR: GitHub release {release_tag.name!r} does NOT exist.")

    # Set up shared state
    state = _State(options, release)

    # Push the requested release artifacts.
    if options.push_source:
        _push_source(state)
    if options.push_s3:
        _push_s3(state)


if __name__ == "__main__":
    main(sys.argv[1:])
