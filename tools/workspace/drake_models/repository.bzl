load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "6bff94f851ed717ea8d04e06e255ea7f5b7a525f",
        sha256 = "947740d9ef7ea41518a5a6b95b60e35391ef7508c14e1c5e1db470507e60a7f6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
