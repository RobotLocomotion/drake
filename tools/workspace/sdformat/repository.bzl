# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    commit = "8db558193cb3"
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "eddd4bbddf2ebee1e832efa95ff00140b5ddff90a253653397a5f299169db74e",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
