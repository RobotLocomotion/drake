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
        commit = "84819c4af986511c08d8cf55735096b9902a8717",
        sha256 = "d88a2bc7ec11608684176357472200a98dc301682be713caa64b2f159ead8a89",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
