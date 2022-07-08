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
        commit = "c940ec07ecb4f109712d2be071798a00646823ca",
        sha256 = "b7718ca30b46c6d4420b3ed2f7890bce998bee659d056954c9051a9cba202404",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
