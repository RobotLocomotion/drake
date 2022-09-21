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
        commit = "2cabdb3595d103552556d1d561c4115ca941333f",
        sha256 = "577995c7e40bb0ef90b28ad950ff5d210be1a797555dd23e6f0062c88fe80fc4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
