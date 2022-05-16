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
        commit = "b771333f92ec34717e8df505d0a50d1faaa458e8",
        sha256 = "4c5d6a346602ba1d97ccd6f979ca4b292550349992c9025372ec6aabb64f7d96",  # noqa
        build_file = "@drake//tools/workspace/voxelized_geometry_tools:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
