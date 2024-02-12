load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "8cdc23d313c997d7f317de0c6b36014e2906f3c0",
        sha256 = "f1b37fb0f82be4b221bac0497d62cc4505afd8ddf9f46062e97f5797b8d73baa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
