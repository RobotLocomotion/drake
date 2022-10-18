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
        commit = "985264e77ea9524a993b129e913655223a782cc7",
        sha256 = "c7bdcef50b37eb99d85215f00c9d85c36d3d728d8e5cca434e726966fe9da5b3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
