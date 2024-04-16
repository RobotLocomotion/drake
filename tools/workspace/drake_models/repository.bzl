load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "499653bb801a8cd1e1bf2350fec3bbff5e4fb87a",
        sha256 = "05913fdea320a766a8686fff80120f9251ac27b483aa23d4d37bebec9b02fbc9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
