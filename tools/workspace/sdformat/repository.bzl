# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "e5fd14d953aa",
        sha256 = "4054ff905a81013ba1da33f40bc35004ef9db75ae122bdfb755a03c723c6e8ac",  # noqa
        strip_prefix = "osrf-sdformat-e5fd14d953aa",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
