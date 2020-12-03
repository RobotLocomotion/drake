# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "983e8fae73ace20897c97dffacbd00fb0ec01ccc",
        sha256 = "ec59fbf1ae8bf48c1fd99c8000d5d54912d2078b94df32c6a09ff188500e72b0",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
