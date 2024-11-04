load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "47a987adcc9521bea2c300fee2c334bd5950d3eb",
        sha256 = "49284e3afc7c6ccd80b0c605f070c53ef0bb5d1d96330cf15e87c68d804e78a8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
