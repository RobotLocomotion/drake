"""Mirrors artifacts from the most recent Drake release to the drake-packages
bucket on Amazon S3. Requires suitable AWS credentials to be configured per
https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html.

This script is neither called during the build nor expected to be called by
most developers or users of the project. It is only supported when run under
Python 3 on Ubuntu.

To run:

    bazel build //tools/release_engineering:mirror_to_s3
    bazel-bin/tools/release_engineering/mirror_to_s3 \
        [--dry-run] [--token <token>]
"""

import argparse
import os
import sys
import tempfile

import boto3
import botocore
import github3
from github3.repos.release import Asset, Release

_GITHUB_REPO_OWNER = "RobotLocomotion"
_GITHUB_REPO_NAME = "drake"

_AWS_BUCKET = "drake-packages"
_AWS_PREFIX = "drake/release"


class _State:
    def __init__(self, release: Release, dry_run: bool):
        self._release = release
        self._dry_run = dry_run
        self._scratch = tempfile.TemporaryDirectory()
        self._s3 = boto3.resource("s3")

    def get_artifacts(self) -> list[Asset]:
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
        Pushes the specified artifact to S3 under the given 'bucket'/'path',
        if it doesn't already exist there.

        If --dry-run was given, rather than actually pushing files to S3,
        prints what would be done.
        """
        dest = f"s3://{bucket}/{path}"
        s3_object = self._s3.Object(bucket, path)
        try:
            s3_object.load()
            print(f"S3 object key {path} already exists")
        except botocore.exceptions.ClientError as exception:
            if exception.response["Error"]["Code"] in ["403", "404"]:
                print(f"S3 object key {path} does not exist")
            print(
                f"push {artifact.name!r} ({artifact.content_type})"
                f" to {dest!r}",
                flush=True,
            )
            local_path = os.path.join(self._scratch.name, artifact.name)
            if not self._dry_run:
                assert artifact.download(path=local_path) is not None
            print(" done with download", flush=True)
            if not self._dry_run:
                s3_object.upload_file(local_path)
            print(" done with upload", flush=True)


def _push_s3(state: _State) -> None:
    """
    Downloads GitHub release artifacts and pushes them to S3.
    """
    for artifact in state.get_artifacts():
        dest_path = f"{_AWS_PREFIX}/{artifact.name}"
        state.push_artifact(artifact, _AWS_BUCKET, dest_path)


def main(args: list[str]) -> None:
    parser = argparse.ArgumentParser(prog="push_release", description=__doc__)
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
        help="Use the GitHub API token read from this path, if it exists."
        " Otherwise, anonymous access will be used, which may run into"
        " rate limitations. (default: %(default)s)",
    )
    options = parser.parse_args(args)

    # Get GitHub repository and release object.
    with open(os.path.expanduser(options.token), "r") as f:
        token = f.read().strip()
    gh = github3.login(token=token)
    repo = gh.repository(_GITHUB_REPO_OWNER, _GITHUB_REPO_NAME)
    release = repo.latest_release()

    # Set up shared state.
    state = _State(release, options.dry_run)

    # Push the requested release artifacts.
    _push_s3(state)


if __name__ == "__main__":
    main(sys.argv[1:])
