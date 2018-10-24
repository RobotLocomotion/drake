# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "43c23baba4d9f8734ae8419b45fc81fee99b3c4e",
        sha256 = "ad9d12161d8a5bb974ed537da4031559107d11f033451666be432211fa007f4c",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
