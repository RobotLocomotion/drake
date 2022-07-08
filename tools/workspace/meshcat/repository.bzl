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
        commit = "65781fcb064db536b99a66fe9fcf5bf0b6d1f790",
        sha256 = "d55918a9d14b1f92b331e5d64df7be1572c670afcc4dd5f46372c186708b9e80",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
