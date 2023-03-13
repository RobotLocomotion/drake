load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "6751ba850fd46fee789a095edba7ad9fc99cd188",
        sha256 = "dc38658f7acb4d07442e6dae4d18818d31ac6cc6d66bf16913ef1b208692627c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
