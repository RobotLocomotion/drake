load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "9eec25c931a1f78f75ae2996c3209dac0a172445",
        sha256 = "5efcacc08a47b395e2c91ef0353b0a0c3d2b9ec06d6a161078ff9baaadc364ff",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
