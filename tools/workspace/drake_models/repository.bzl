load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "7301acb8fcaf849a8c74ef4d1d76bf56bec628ef",
        sha256 = "3d7b43b4b5b7586f0db2ffee76c6a54c010052f081d60545d1f451399f9b8006",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
