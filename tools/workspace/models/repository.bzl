# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "527933b8687cdc54c36a22c5ce2541192baa6731",
        sha256 = "c545096cda709ee994265663c98045c2bd0bda0134ddf7b399294f9c716d80aa",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
