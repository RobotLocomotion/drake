# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sammy-tri/models",
        commit = "a62b83c2788aee882b5c8ad6e4106053f0e7078e",
        sha256 = "809adb073ba4b9094cca083458dbc56e4f129a6ab4e6e8fd0ab924a0e4712b22",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
