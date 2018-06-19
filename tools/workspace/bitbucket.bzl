# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def bitbucket_archive(
        name,
        repository = None,
        commit = None,
        sha256 = None,
        strip_prefix = None,
        build_file = None,
        mirrors = None,
        **kwargs):
    """A macro to be called in the WORKSPACE that adds an external from
    bitbucket using a workspace rule.

    The required name= is the rule name and so is used for @name//... labels
    when referring to this archive from BUILD files.

    The required commit= is the hash to download.  (When a git project is
    also a git submodule in CMake, this should be kept in sync with the git
    submodule commit used there.)  This can also be a tag.

    The required sha256= is the checksum of the downloaded archive.  When
    unsure, you can omit this argument (or comment it out) and then the
    checksum-mismatch error message message will offer a suggestion.

    The required build_prefix= is the directory prefix to strip from the
    extracted files.

    The optional build_file= is the BUILD file label to use for building this
    external.  When omitted, the BUILD file(s) within the archive will be used.

    The required mirrors= is a dict from string to list-of-string with key
    "bitbucket", where the list-of-strings are URLs to use, formatted using
    {repository} and {commit} string substitutions.  The mirrors.bzl file
    in this directory provides a reasonable default value.
    """
    if repository == None:
        fail("Missing repository=")
    if commit == None:
        fail("Missing commit=")
    if sha256 == None or len(sha256) == 0:
        # This is mostly-required, but we fallback to a wrong-default value to
        # allow the first attempt to fail and print the correct sha256.
        sha256 = "0" * 64
    if strip_prefix == None:
        fail("Missing strip_prefix=")
    if mirrors == None:
        fail("Missing mirrors=; see mirrors.bzl")

    urls = [
        x.format(repository = repository, commit = commit)
        for x in mirrors.get("bitbucket")
    ]

    repository_split = repository.split("/")
    if len(repository_split) != 2:
        fail("The repository= must be formatted as 'organization/project'")

    http_archive(
        name = name,
        urls = urls,
        sha256 = sha256,
        build_file = build_file,
        strip_prefix = strip_prefix,
        **kwargs
    )
