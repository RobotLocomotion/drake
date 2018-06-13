# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    commit = "7de28392a8c2"
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "fcc149fe8ade380a201263d1f98a65bb121f6c37726b6d084e86e9a8c7695ab1",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
