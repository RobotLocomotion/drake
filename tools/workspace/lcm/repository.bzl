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
        commit = "91ce7a2ae46ad05f8a232f5fe32a06cccbead1c2",
        sha256 = "8ea0076d2f2158fc750fec697b68c6903a9d70ccbe4e3f24240415a13445381f",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
