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
        commit = "8b1008197f6a77811a572db8de45b125da216df1",
        sha256 = "034322a87723536ceb79b9239e942a8993defbfd22be89940bb2f69da5361e11",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
