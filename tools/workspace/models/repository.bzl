# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "d9b1abebba788cb04dbfc208b48de5dc12e742d3",
        sha256 = "d8a0c8b7b4dea2621af5e0f7bdbec47966c9ca5e7b902fa79657bbe6ef763784",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
