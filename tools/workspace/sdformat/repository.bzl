# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        commit = "0aa8eeaa12df1fabc08cac7c6ebc08c49a1927ec",
        sha256 = "414835f78bd442fae2156dd7b6b41a734c4631f5c4df9eae483f8b253980591c",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
