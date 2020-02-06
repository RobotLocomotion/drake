# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "e4e00226386846716e8244efa8e05c3e74e90c7f",
        sha256 = "17f6ca8e7e204bf36e20ba82a4d6f452e2d5e28db77b5e0a79a0dc5502b585c1",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
