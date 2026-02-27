load("//tools/workspace:github.bzl", "github_archive")

def styleguide_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "b87173bc4d62e9662a73ceb3f7f34e61c87334a0",
        sha256 = "64ac6a6d71e10a006275ee3b8f557b45bdf174ee9a00282623024451ebc8c4cf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
