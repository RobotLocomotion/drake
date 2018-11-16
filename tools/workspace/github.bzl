# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def github_archive(
        name,
        repository = None,
        commit = None,
        sha256 = "0" * 64,
        build_file = None,
        local_repository_override = None,
        mirrors = None,
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from GitHub
    using a workspace rule.

    Args:
        name: required name is the rule name and so is used for @name//...
            labels when referring to this archive from BUILD files.
        repository: required GitHub repository name in the form
            organization/project.
        commit: required commit is the git hash to download. (When the git
            project is also a git submodule in CMake, this should be kept in
            sync with the git submodule commit used there.) This can also be a
            tag.
        sha256: required sha256 is the expected SHA-256 checksum of the
            downloaded archive. When unsure, you can omit this argument (or
            comment it out) and then the checksum-mismatch error message will
            offer a suggestion.
        build_file: optional build file is the BUILD file label to use for
            building this external. When omitted, the BUILD file(s) within the
            archive will be used.
        local_repository_override: optional local repository override can be
            used for temporary local testing; instead of retrieving the code
            from GitHub, the code is retrieved from the local filesystem path
            given in the argument.
        mirrors: required mirrors is a dict from string to list-of-string with
            key "github", where the list-of-strings are URLs to use, formatted
            using {repository} and {commit} string substitutions. The
            mirrors.bzl file in this directory provides a reasonable default
            value.
    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if mirrors == None:
        fail("Missing mirrors=; see mirrors.bzl")

    if local_repository_override != None:
        if build_file == None:
            native.local_repository(
                name = name,
                path = local_repository_override,
            )
        else:
            native.new_local_repository(
                name = name,
                build_file = build_file,
                path = local_repository_override,
            )
        return

    http_archive(
        name = name,
        urls = _urls(repository, commit, mirrors),
        sha256 = _sha256(sha256),
        build_file = build_file,
        strip_prefix = _strip_prefix(repository, commit),
        **kwargs
    )

def github_download_and_extract(
        repository_ctx,
        repository,
        commit,
        mirrors,
        output = "",
        sha256 = "0" * 64):
    """Download an archive of the provided GitHub repository and commit to the
    output path and extract it.

    Args:
        repository_ctx: context of a Bazel repository rule.
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive should be downloaded.
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
        stripPrefix = _strip_prefix(repository, commit),
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

def _strip_prefix(repository, commit):
    """Compute the strip prefix for a downloaded archive of the provided
    GitHub repository and commit.

    Args:
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive was downloaded.
    """
    repository_split = repository.split("/")

    if len(repository_split) != 2:
        fail("repository must be formatted as organization/project")

    _, project = repository_split

    # GitHub archives omit the "v" in version tags, for some reason.
    if commit[0] == "v":
        strip_commit = commit[1:]
    else:
        strip_commit = commit

    return project + "-" + strip_commit

def _urls(repository, commit, mirrors):
    """Compute the urls from which an archive of the provided GitHub
    repository and commit may be downloaded.

     Args:
        repository: GitHub repository name in the form organization/project.
        commit: git revision for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
    """
    return [
        x.format(
            repository = repository,
            commit = commit,
        )
        for x in mirrors.get("github")
    ]
