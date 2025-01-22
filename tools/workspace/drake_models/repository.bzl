load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "11e0cc1384b32aa808ab8805153ec3432b472bbf",
        sha256 = "4a53240a0218a6b2c63d68d5f7bce287d9a2715637fd8b97ccc23c869b1b97fd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
