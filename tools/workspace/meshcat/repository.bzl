# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        # Updating this commit requires local testing; see
        # drake/tools/workspace/meshcat/README.md for details.
        commit = "a03d1abf636c059166132a6d9434a164a155ecbe",
        sha256 = "b23fcc5152c0eafc51a2255928a93568151f93e0f812fbc7e7fc3548f2e9c49a",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
