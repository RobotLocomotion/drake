load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "5a81de1db0570494654db61e1cb5fa4aa26ad503",
        sha256 = "830d8b497f6cda16b21de10936d3111143d850672e6e22d2d6f8862db0697b0e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
