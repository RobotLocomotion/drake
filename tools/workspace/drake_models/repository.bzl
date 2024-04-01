load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "a70350137add3287527bee2dfb3cad21e9c363c2",
        sha256 = "2bf1417c9e7c4e581ae46a2c130199f2b5b14a7eb372131d56e653e84b267daf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
