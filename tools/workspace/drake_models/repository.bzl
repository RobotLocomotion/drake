load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "3de997ac1dbb4152de09dd0890584ce279400cdb",
        sha256 = "123e8ac9d34e96c8a4ce3495483d9d90ac27ae31f0843166fe916fa2ae5f62af",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
