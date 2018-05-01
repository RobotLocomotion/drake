# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "1f508738ee8e20ff6193e98b5b77b2644b7dc269",
        sha256 = "2ee2e35b249d6f6fa25010374f6b3e5ba8917ba756566c85b1f7b3841d3d2210",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
