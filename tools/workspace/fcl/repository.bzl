# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "0d98b836d57b1a4f457fac8e52d6b22b56e5fdbf",
        sha256 = "e658fb6a6c3f5e9ee4080e473afdc3c8163df0f58847284439052d065cd7a38b",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
