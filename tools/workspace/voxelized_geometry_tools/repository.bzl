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
        commit = "e1ac32f276aceb0596bc5afd79ab02a5b3cbe457",
        sha256 = "63a478d3492b68fe2b23aa04778755467b7076db6301942fdc4d0eeb74837586",  # noqa
        build_file = "//tools/workspace/voxelized_geometry_tools:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
