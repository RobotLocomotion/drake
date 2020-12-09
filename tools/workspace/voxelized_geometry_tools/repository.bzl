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
        commit = "374afb7e3b2b3d1610ce82ecad001c2d4558ae3c",
        sha256 = "a6a6952f9e5d512bafd1a244b65ae83ec676beb61511fba203f5ed1a692820d3",  # noqa
        build_file = "//tools/workspace/voxelized_geometry_tools:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
