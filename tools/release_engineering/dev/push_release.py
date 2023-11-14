"""
Uploads Drake release artifacts.

This is intended for use by Drake maintainers (only).
This program is only supported on Ubuntu Jammy 22.04.
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Dict, List

import boto3

import docker

import github3
from github3.repos.release import Asset, Release
from github3.repos.repo import Repository


_GITHUB_REPO_OWNER = 'RobotLocomotion'
_GITHUB_REPO_NAME = 'drake'

_ARCHIVE_HASHES = {'sha256', 'sha512'}

_DOCKER_REGISTRY_API_URI = 'https://registry.hub.docker.com/v2/repositories'
_DOCKER_REPOSITORY_NAME = 'robotlocomotion/drake'

_AWS_BUCKET = 'drake-packages'


@dataclass
class _Artifact:
    name: str
    ext: str
    asset: Asset
    version: str
    platform: str
    arch: str = None
    hashes: Dict[str, Asset] = field(default_factory=dict)


class _Manifest:
    """
    Regex matching a tarball, e.g. 'drake-0.1.0-mac-arm64.tar.gz'.
    """
    RE_TAR = re.compile(
        r'^drake-'
        r'(?P<id>[0-9.]+)-'
        r'(?P<platform>\w+)'
        r'(-(?P<arch>\w+))?.'
        r'(?P<ext>tar.[gx]z)$')
    """
    Regex matching a .deb package, e.g. 'drake-dev_0.1.0-1_amd64-jammy.deb'.
    """
    RE_DEB = re.compile(
        r'^drake-dev_'
        r'(?P<id>[^-]+-[0-9]+)_'
        r'(?P<arch>\w+)-'
        r'(?P<platform>\w+).'
        r'(?P<ext>deb)$')

    def __init__(self, release: Release):
        self._assets = list(release.assets())

    def _find_hashes(self, name) -> Dict[str, Asset]:
        """
        Finds hashes associated with an asset of the given name.
        """
        pre = f'{name}.sha'
        result = {}

        for a in self._assets:
            if a.name.startswith(pre):
                result[a.name[len(pre):]] = a

        return result

    def find_artifacts(self, regex: re.Pattern) -> List[_Artifact]:
        """
        Finds assets whose name matches the given regular expression.

        The regular expression must be one of RE_TAR or RE_DEB members
        of this class.
        """
        result = []

        for a in self._assets:
            m = regex.match(a.name)
            if m is not None:
                attrs = m.groupdict()
                n = attrs.pop('id')

                artifact = _Artifact(name=a.name, asset=a, version=n, **attrs)
                artifact.hashes = self._find_hashes(a.name)
                result.append(artifact)

        return result


@dataclass
class _State:
    options: Dict[str, Any]
    release: Release
    manifest: _Manifest

    def __init__(self, options: Dict[str, Any], release: Release):
        self.options = options
        self.release = release
        self.manifest = _Manifest(release)
        self._scratch = tempfile.TemporaryDirectory()
        self._s3 = boto3.client('s3')
        self._docker = docker.APIClient()

        self.find_artifacts = self.manifest.find_artifacts

    def _begin(self, action: str, src: str, dst: str = None):
        """
        Report the start of an action.
        """
        if dst is not None:
            print(f'{action} {src!r} to {dst!r} ...', flush=True, end='')
        else:
            print(f'{action} {src!r} ...', flush=True, end='')

    def _done(self):
        """
        Report the completion of an action.
        """

    def _push_asset(self, asset: Asset, bucket: str, path: str):
        """
        Pushes the specified asset to S3.

        If --dry-run was given, rather than actually pushing files to S3,
        prints what would be done.
        """
        if self.options.dry_run:
            print(f'push {asset.name!r} to s3://{bucket}/{path}')
        else:
            self._begin('downloading', asset.name)
            local_path = os.path.join(self._scratch.name, asset.name)
            assert asset.download(path=local_path) is not None
            self._done()

            self._begin('pushing', asset.name, f's3://{bucket}/{path}')
            self._s3.upload_file(local_path, bucket, path)
            self._done()

    def _compute_hash(self, path: str, algorithm: str):
        """
        Compute the specified hash for the specified file.
        """
        with open(path, 'rb') as f:
            if hasattr(hashlib, 'file_digest'):
                return hashlib.file_digest(f, algorithm)

            else:
                digest = hashlib.new(algorithm)
                buf = bytearray(256 * 1024)
                view = memoryview(buf)

                while True:
                    size = f.readinto(buf)
                    if size == 0:
                        break
                    digest.update(view[:size])

                return digest

    def push_artifact(self, artifact: _Artifact, bucket: str,
                      path: str, include_hashes: bool = True):
        """
        Pushes the specified artifact to S3, optionally including any
        associated hashes.

        If --dry-run was given, rather than actually pushing files to S3,
        prints what would be done.
        """
        self._push_asset(artifact.asset, bucket, path)
        if include_hashes:
            for h, a in artifact.hashes.items():
                self._push_asset(a, bucket, f'{path}.sha{h}')

    def push_archive(self, bucket: str, path: str, name: str,
                     archive_format: str = 'tarball'):
        """
        Pushes the release source archive to S3, along with computed hashes.

        If --dry-run was given, rather than actually pushing files to S3,
        prints what would be done.
        """
        local_path = os.path.join(self._scratch.name, name)
        self._begin('downloading', name)
        assert self.release.archive(archive_format, local_path)
        self._done()

        if self.options.dry_run:
            print(f'push {name!r} to s3://{bucket}/{path}/{name}')
        else:
            self._begin('pushing', name, f's3://{bucket}/{path}/{name}')
            self._s3.upload_file(local_path, bucket, f'{path}/{name}')
            self._done()

        # Calculate hashes and write hash files
        for algorithm in _ARCHIVE_HASHES:
            hashfile_name = f'{name}.{algorithm}'
            hashfile_path = os.path.join(self._scratch.name, hashfile_name)
            remote_path = f'{path}/{hashfile_name}'

            self._begin(f'computing {algorithm} for', name)

            digest = self._compute_hash(local_path, algorithm)

            with open(hashfile_path, 'wt') as f:
                f.write(f'{digest.hexdigest()}  {name}\n')

            self._done()

            if self.options.dry_run:
                print(f'{name!r} {algorithm}: {digest.hexdigest()}')
                print(f'push {hashfile_name!r} to s3://{bucket}/{remote_path}')
            else:
                self._begin('pushing', hashfile_name,
                            f's3://{bucket}/{remote_path}')
                self._s3.upload_file(hashfile_path, bucket, remote_path)
                self._done()

    def push_docker_tag(self, old_tag_name: str, new_tag_name: str,
                        repository: str = _DOCKER_REPOSITORY_NAME):
        image = f'{repository}:{old_tag_name}'
        if self.options.dry_run:
            print(f'push {image!r} to {repository!r} as {new_tag_name!r}')
        else:
            self._begin('pulling', f'{repository}:{old_tag_name}')
            self._docker.pull(repository, old_tag_name)
            self._done()

            self._docker.tag(image, repository, new_tag_name)

            self._begin('pushing', f'{repository}:{new_tag_name}')
            self._docker.push(repository, new_tag_name)
            self._done()

            self._docker.remove_image(image)


def _fatal(msg: str, result: int = 1):
    width = shutil.get_terminal_size().columns
    for line in msg.split('\n'):
        print(textwrap.fill(line, width), file=sys.stderr)
    sys.exit(result)


def _assert_tty():
    try:
        subprocess.check_call(['tty', '-s'])
    except subprocess.CalledProcessError:
        _fatal('ERROR: tty was NOT detected. This script may need'
               ' various login credentials to be entered interactively.')
        sys.exit(1)


def _assert_command_exists(name: str, package: str):
    """
    Asserts that an executable <name> exists,
    or tells the user to install <package>.
    """
    if shutil.which(name) is None:
        _fatal(f'ERROR: `{name}` was not found. '
               f'Fix with `apt-get install {package}`.')


def _test_non_empty(path):
    """
    Tests if the specified path exists and is a non-empty file.
    """
    if path is None:
        return False

    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return False

    return os.stat(path).st_size > 0


def _check_version(version):
    """
    Returns True iff the given version string matches PEP 440.
    """
    return re.match(
        r'^([1-9][0-9]*!)?(0|[1-9][0-9]*)'
        r'(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?'
        r'(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?'
        r'([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$',
        version) is not None


def _find_tag(repo: Repository, tag: str):
    """
    Finds the tag <tag> in the repository <repo>.
    """
    for t in repo.tags():
        if t.name == tag:
            return t
    return None


def _list_docker_tags(repository=_DOCKER_REPOSITORY_NAME):
    tags = []
    uri = f'{_DOCKER_REGISTRY_API_URI}/{repository}/tags?page_size=1000'

    while uri is not None:
        with urllib.request.urlopen(uri) as response:
            reply = json.load(response)

        for t in reply['results']:
            tags.append(t['name'])

        uri = reply.get('next')

    return tags


def _push_tar(state: _State):
    """
    Downloads .tar artifacts and push them to S3.
    """
    version = state.options.source_version
    for tar in state.find_artifacts(_Manifest.RE_TAR):
        state.push_artifact(tar, _AWS_BUCKET, f'drake/release/{tar.name}')

    dest_name = f'drake-{version}-src.tar.gz'

    # TODO(mwoehlke-kitware):
    # Eventually, the source tarball should be a discrete asset of the GitHub
    # release, including hash files.
    state.push_archive(_AWS_BUCKET, 'drake/release', dest_name)


def _push_deb(state: _State):
    """
    Downloads .deb artifacts and push them to S3.
    """
    for deb in state.find_artifacts(_Manifest.RE_DEB):
        dest_name = f'drake-dev_{deb.version}_{deb.arch}.{deb.ext}'
        dest_path = f'drake/release/{deb.platform}/{dest_name}'
        state.push_artifact(deb, _AWS_BUCKET, dest_path)


def _push_docker(state: _State):
    """
    Re-tags Docker staging images as release images.
    """
    tail = f'{state.options.source_version}-staging'
    for tag_name in _list_docker_tags():
        if tag_name.endswith(tail):
            release_tag_name = tag_name.rsplit('-', 1)[0]
            state.push_docker_tag(tag_name, release_tag_name)


def main(args: List[str]):
    parser = argparse.ArgumentParser(
        prog='push_release', description=__doc__)
    parser.add_argument(
        '--deb', dest='push_deb', default=True,
        action=argparse.BooleanOptionalAction,
        help='Mirror .deb packages to S3.')
    parser.add_argument(
        '--docker', dest='push_docker', default=True,
        action=argparse.BooleanOptionalAction,
        help='Publish docker images from staging images.')
    parser.add_argument(
        '--tar', dest='push_tar', default=True,
        action=argparse.BooleanOptionalAction,
        help='Mirror .tar packages to S3.')
    parser.add_argument(
        '-n', '--dry-run', default=False,
        action=argparse.BooleanOptionalAction,
        help='Print what would be done without actually pushing anything.')
    parser.add_argument(
        '--token', default='~/.config/readonly_github_api_token.txt',
        help='Use the GitHub an API token read from this path, if it exists.'
             ' Otherwise, anonymous access will be used, which may run into'
             ' rate limitations. (default: %(default)s)')
    parser.add_argument(
        'source_version',
        help='Version tag (x.y.z) of the release to be pushed.')
    options = parser.parse_args(args)

    # Validate version arguments
    if not _check_version(options.source_version):
        _fatal(f'ERROR: source_version {options.source_version!r}'
               ' is not a valid version number.')

    # Ensure execution environment is suitable.
    _assert_tty()
    if options.push_tar or options.push_deb:
        if not _test_non_empty('~/.aws/credentials'):
            _fatal('ERROR: AWS credentials were not found.\n\n'
                   'The --tar and/or --deb options require the ability'
                   ' to push files to S3, which requires authentication'
                   ' credentials to be provided.\n\n'
                   'Fix this by running `aws configure`.')

    # Get GitHub repository, release tag, and release object.
    if _test_non_empty(options.token):
        with open(os.path.expanduser(options.token), 'r') as f:
            token = f.read().strip()
        gh = github3.login(token=token)
    else:
        gh = github3.GitHub()
    repo = gh.repository(_GITHUB_REPO_OWNER, _GITHUB_REPO_NAME)
    release_tag = _find_tag(repo, f'v{options.source_version}')

    if release_tag is None:
        _fatal(f'ERROR: GitHub tag v{options.source_version}'
               ' does NOT exist.')

    release = repo.release_from_tag(release_tag)

    if release_tag is None:
        _fatal(f'ERROR: GitHub release {release_tag.name!r} does NOT exist.')

    # Set up shared state
    state = _State(options, release)

    # Push the requested release artifacts.
    if options.push_tar:
        _push_tar(state)
    if options.push_deb:
        _push_deb(state)
    if options.push_docker:
        _push_docker(state)


if __name__ == '__main__':
    main(sys.argv[1:])
