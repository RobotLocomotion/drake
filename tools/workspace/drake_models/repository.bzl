load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        upgrade_type = "commit",
        commit = "f85076b4d95a8bdfec70153015e723e92312e4e2",
        sha256 = "67f5684aed2dca280e7a93fdf8b6555e29d9c3831cf5176639a03530b78f165a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
