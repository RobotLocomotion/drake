# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bullet_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bulletphysics/bullet3",
        commit = "2.88",
        sha256 = "21c135775527754fc2929db1db5144e92ad0218ae72840a9f162acb467a7bbf9",  # noqa
        build_file = "@drake//tools/workspace/bullet:package.BUILD.bazel",
        mirrors = mirrors,
    )
