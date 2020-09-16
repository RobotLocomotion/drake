# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "a7d30f8a4682bf94f897711dae41ef581027a358",
        sha256 = "1ad561d4d95846c5277710a8fbba16fb8acd7750f713fd4d81b3764933c381a1",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
