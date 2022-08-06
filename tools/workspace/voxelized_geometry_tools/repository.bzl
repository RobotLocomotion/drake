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
        # When updating, ensure that any new unit tests are reflected in
        # package.BUILD.bazel and BUILD.bazel in drake.
        commit = "44193d05955406baeb9829556ea8d67f5529feff",
        sha256 = "b5d91c1255fa9fd8e3be06196c6ff4a0105b4d2325d276429cec3dc35fb12d9e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
