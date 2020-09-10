# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "41f4e001996ece2590f3a618e4481da1d9d93362",
        sha256 = "e88293fa0f4582ffca8b9862b6a2556e666dee8b7f447ec1c71bf7062e5c800f",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
