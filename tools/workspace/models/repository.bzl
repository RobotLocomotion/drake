# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "70593dc411979f411d55360a28ea32f1242c37dd",
        sha256 = "ec3db4d37948fdd72c7d6aa6797db3945441bfaf90d454c609afbc3710f24432",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
