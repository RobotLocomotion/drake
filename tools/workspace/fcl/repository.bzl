# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "08d0bb25c290a4a5e713851d2a5ac514e4355801",
        sha256 = "1b8213a2c25fbcf8a9dbc0fa45bf32c7de222671fafc0bdcdfe248b95dc4c56b",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
