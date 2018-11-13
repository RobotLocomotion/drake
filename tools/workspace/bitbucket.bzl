# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def bitbucket_archive(
        name,
        repository = None,
        commit = None,
        sha256 = "0" * 64,
        strip_prefix = None,
        build_file = None,
        mirrors = None,
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from
    bitbucket using a workspace rule.

    Args:
        name: required name is the rule name and so is used for @name//...
            labels when referring to this archive from BUILD files.
        repository: required Bitbucket repository name.
        commit: required commit is the hash to download. (When a git project
            is also a git submodule in CMake, this should be kept in sync with
            the git submodule commit used there.) This can also be a tag.
        sha256: required sha256 is the expected SHA-256 checksum of the
            downloaded archive. When unsure, you can omit this argument (or
            comment it out) and then the checksum-mismatch error message will
            offer a suggestion.
        strip_prefix: required strip prefix is the directory prefix to strip
            from the extracted files.
        build_file: optional build file is the BUILD file label to use for
            building this external. When omitted, the BUILD file(s) within the
            archive will be used.
        mirrors: required mirrors is a dict from string to list-of-string with
            key "bitbucket", where the list-of-strings are URLs to use,
            formatted using {repository} and {commit} string substitutions.
            The mirrors.bzl file in this directory provides a reasonable
            default value.
    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if strip_prefix == None:
        fail("Missing strip_prefix=")
    if mirrors == None:
        fail("Missing mirrors=; see mirrors.bzl")

    repository_split = repository.split("/")
    if len(repository_split) != 2:
        fail("The repository= must be formatted as 'organization/project'")

    http_archive(
        name = name,
        urls = _urls(repository, commit, mirrors),
        sha256 = _sha256(sha256),
        build_file = build_file,
        strip_prefix = strip_prefix,
        **kwargs
    )

def bitbucket_download_and_extract(
        repository_ctx,
        repository,
        commit,
        mirrors,
        output = "",
        sha256 = "0" * 64,
        strip_prefix = ""):
    """Download an archive of the provided Bitbucket repository and commit to
    the output path and extract it.

    Args:
        repository_ctx: context of a Bazel repository rule.
        repository: Bitbucket repository name.
        commit: revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
        output: path to the directory where the archive will be unpacked,
            relative to the Bazel repository directory.
        sha256: expected SHA-256 hash of the archive downloaded. Fallback to
            an incorrect default value to prevent the hash check from being
            disabled, but allow the first download attempt to fail and print
            the correct SHA-256 hash.
    """
    urls = _urls(repository, commit, mirrors)

    repository_ctx.download_and_extract(
        urls,
        output = output,
        sha256 = _sha256(sha256),
        stripPrefix = strip_prefix,
    )

def _sha256(sha256):
    """Fallback to an incorrect default value of SHA-256 hash to prevent the
    hash check from being disabled, but allow the first download attempt of an
    archive to fail and print the correct hash.

    Args:
        sha256: expected SHA-256 hash of the archive to be downloaded.
    """
    if not sha256:
        sha256 = "0" * 64

    return sha256

def _urls(repository, commit, mirrors):
    """Compute the urls from which an archive of the provided Bitbucket
    repository and commit may be downloaded.

     Args:
        repository: Bitbucket repository name.
        commit: revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
    """
    return [
        x.format(
            repository = repository,
            commit = commit,
        )
        for x in mirrors.get("bitbucket")
    ]
