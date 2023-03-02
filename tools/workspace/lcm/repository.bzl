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
        commit = "1aecca45e6a05d719da8e566533e45740d1fd88c",
        sha256 = "78ef84ccabf78ebb33a182393bb539f38fedb576e2c1cb244bf134470bd702e3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
