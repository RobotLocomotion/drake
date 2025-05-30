load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "0c32f2ecab1a0c735e8381bf98659bfff3a18fd5",
        sha256 = "6b656451b020719b7fc3eec25f6e1bde8bebbb142bb0e2c4f560ae35c79a334c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
