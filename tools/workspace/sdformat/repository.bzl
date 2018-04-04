# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "ac1aff4f7207",
        sha256 = "7d69ca086ef9ea4a495130c8fa1a7a1409a24e497f8c08a6467e8a2bbfa445d3",  # noqa
        strip_prefix = "osrf-sdformat-ac1aff4f7207",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
