load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "7ede868f6dd76f1a68c9dc34da991e16ba61dfca",
        sha256 = "e32cda48618fefd28dca06044916c4aad1b46a9de0c410835b5ccb46581826d1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
