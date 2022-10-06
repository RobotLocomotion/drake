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
        commit = "d1049e0d463cf3f5918ef6a3a714c894038b7d79",
        sha256 = "caf1bbd6bea11ddfa468d0307155695e9500a2f67e04416a423c5bd376e38151",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
