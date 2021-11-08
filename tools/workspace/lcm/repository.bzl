# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        # When upgrading this commit, check if the LCM maintainers have tagged
        # a new version number; if so, then update the version numbers within
        # the two lcm-*.cmake files in this directory to match.
        commit = "e83d2d0810c7f1751123383ab7c3dc4da1b53602",
        sha256 = "13f478db7002165e9987b6e6cbcc8e42e2c103c882b360236b83c9ba5355f539",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
