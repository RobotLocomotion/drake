# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "1378086404d5135b7d49c60252474e480f8bd6aa",
        sha256 = "26d7de535be77927b6be521b883dee396a13e3f32443ecc16b13ca57ef9e618f",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
