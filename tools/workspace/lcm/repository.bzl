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
        commit = "abdd8a292fcaf6e331f0449778e275890e12811a",
        sha256 = "6f519938de3dc57878e8de7618f21a254e99245ae9ebeb4699d425628e4818a5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
