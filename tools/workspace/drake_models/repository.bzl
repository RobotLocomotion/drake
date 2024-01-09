load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "02b279d280e98041ada2185e69e4703fd12a54c8",
        sha256 = "f0605f72977750118cbadef241bb33aa28e1eaaf9b4d53d905cccd4315368cbf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
