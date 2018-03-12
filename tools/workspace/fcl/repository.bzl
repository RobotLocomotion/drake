# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        local_repository_override = "/home/amcastro/FCL/fcl",
        commit = "e683dfed7fecc617261e0308250fad8b7ea1d277",
        sha256 = "e88785232d3a7df64b8319ec8a045afc5de10c8bc8ec7fc04fa3ef4118c100e0",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
