load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "d3e98cb2ff293af75239fa09cb8850517b41ed9a",
        sha256 = "9d5b6d8f5388d5db6f5a9c395da6c1be2dff8defb0c00a92575760cb42b52c5d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
