# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "501e47e090fbff24c71fedf762336d9312b58709",
        sha256 = "3513779c40eecfa00168c779c77b36d57461defbbf01c92c096a9e26284efc1b",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
