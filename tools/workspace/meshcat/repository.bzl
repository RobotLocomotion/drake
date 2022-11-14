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
        commit = "946026bf4b146a3448d976ee855ec5c5fd8f7e9b",
        sha256 = "69b6ac6d3de27cf3ba1f52dd717a7171374a4b3d5b27a4d3816c7b4dc7af45dc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
