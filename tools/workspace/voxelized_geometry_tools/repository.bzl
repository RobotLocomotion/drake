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
        repository = "calderpg/voxelized_geometry_tools",
        commit = "878d83bb001736254bea008e1fd1ebf2486800c2",
        sha256 = "8844316ce6df165b158e419823e3ac64ae665b341ef7067cf65adaba12ab42b5",  # noqa
        build_file = "//tools/workspace/voxelized_geometry_tools:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
