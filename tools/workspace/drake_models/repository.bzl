load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "209d8aece4e3b5d36f0831f95156c2c89b01ea3a",
        sha256 = "1536c240252ac2c976693e40a9effb72b1b143f88aa0693090e4168e2ab37ac0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
