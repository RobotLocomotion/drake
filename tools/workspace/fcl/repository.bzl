# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(name):
    github_archive(
        name = name,
        local_repository_override = "/home/alejandro/FCL/fcl",
        repository = "flexible-collision-library/fcl",
        commit = "43048336c34a01156dc216e8534ffb2788675ddf",
        sha256 = "fd74916b92ed58e77c06097dc18f545462417daa8c96fa8ea2a5c81cd3205917",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
    )
