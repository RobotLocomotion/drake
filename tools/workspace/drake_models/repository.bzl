load("@drake//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "611246c443152946e9dcc901b4f956d89a439a61",
        sha256 = "66ad3b0f85b2aaa823d2e69cdaa980af9c0414cdb829b8224fbaa8a2dc083a7c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
