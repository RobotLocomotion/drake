# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "cd7a0c1fcb461b522f7f1a397c86fa964639462c",
        sha256 = "a3e4db8967546c9cfdc7d73c2f74e3c73d63a248039855f32a4dc680c4f21e7b",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
