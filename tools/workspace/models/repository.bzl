# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "f1357291554434bb230d9d4e149b2a3e7b59602f",
        sha256 = "8f662e260fb1746ca82e74146abebf87350ae11a2224de7aa0a960097bea1287",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
