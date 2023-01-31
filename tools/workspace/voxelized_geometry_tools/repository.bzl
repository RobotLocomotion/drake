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
        commit = "83f9d52218fcd31a91b3cb493bedbad53fdf2bb3",
        sha256 = "2a04e90bb6ffcf34b502422b808a90c729ae495c525594957c3cdc9db4dce04c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
