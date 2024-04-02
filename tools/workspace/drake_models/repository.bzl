load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "db36e7f710accd86a098c0aa2d446976f975ad8d",
        sha256 = "73f0416f28b2e299fcee308452d3d58a39da377d01f23e7c4722960f83c7e001",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
