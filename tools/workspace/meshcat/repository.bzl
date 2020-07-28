# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # TODO(russt): Use rdeits/meshcat again once my fixes are merged.
        repository = "RussTedrake/meshcat",
        # Updating this commit requires local testing; see
        # drake/tools/workspace/meshcat/README.md for details.
        commit = "1c1460169e20bb63bb882321b6c5051766bbdf71",
        sha256 = "e5cf68d4229a49d34a6bba4547478e5157564d292ed25637c9d1c4e0aa5f3fc1",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
