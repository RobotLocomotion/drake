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
        commit = "677127a546984c3c0fb0e42fb694217328f66923",
        sha256 = "227f0c35dfccfca32ef816a3d66fb56c6e80e851b974863337de087838db6b3f",  # noqa
        build_file = "@drake//tools/workspace/voxelized_geometry_tools:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
