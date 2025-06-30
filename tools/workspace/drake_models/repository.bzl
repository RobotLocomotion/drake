load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "f66e4e27e8bfade8c43297e5e8c4dcd575918093",
        sha256 = "4c7310cd9812b812333cc24487bf6f4828db01c6313f29a9e252b0acf689f600",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
