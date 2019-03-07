# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    # From the following branch:
    # https://bitbucket.org/osrf/sdformat/commits/branch/default
    commit = "2fed80e6bc44"
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "b9792f701be807a9f522b9b9b09c521340744ba1bd593d533765b1dc7bb40bb5",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
