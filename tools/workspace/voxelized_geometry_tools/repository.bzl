# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/voxelized_geometry_tools",
        # When updating, ensure that any new unit tests are reflected in
        # package.BUILD.bazel and BUILD.bazel in drake.
        commit = "f4fdbfa75bffa225afda13f5adce6e3edf9538ee",
        sha256 = "a52a869a5050ccf5822a0095e24208d58e29262a334ad0db36193c2457621b92",  # noqa
        build_file = "@drake//tools/workspace/voxelized_geometry_tools:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
