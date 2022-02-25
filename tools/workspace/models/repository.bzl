# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "fcb642c0e6f0cb0ac14280f7324eab4466f219e5",
        sha256 = "c8ea68be89c32061d7294735228ad22f2ed29126e711de1a3a3e01fc4d28be4b",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
