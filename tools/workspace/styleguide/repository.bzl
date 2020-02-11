# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "c400ec647014c0bf050b69247ecf7e78ca233fcb",
        sha256 = "2f3f23f0f9aff9609e869c253c365e54e3311172f0fbc6b9c44347b4868f9681",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
