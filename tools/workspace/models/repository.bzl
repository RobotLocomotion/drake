# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "f9490fb47a97b9b2d147c951ce4575ddb4fa35a8",
        sha256 = "0815b9ced9c6d14daa61d0ead521ee115f8cde05f5823083c84a1ecaced7f327",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
