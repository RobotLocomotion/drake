# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "47265980b6e058e933aa9f9a6b335771a2251ca5",
        sha256 = "a5e65746c1fba49c1000b56143180bac31e86fe2587b34297fbed9a0de9e4fbf",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
