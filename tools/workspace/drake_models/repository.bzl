load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "3a8f1c74105533f4cea97b20afe99b2d1a07f577",
        sha256 = "27686be30f9c8046846392e6aeb0fd3568f96575f976ccdf04dcb80ac64c2cec",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
