# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sammy-tri/models",
        commit = "577b7e85da058322bbb83e1ffe98b52d08ba80dc",
        sha256 = "139e8574f6d87e8c879cd063256ff1ddf8b5f31e1380a00085c2d3fe2b13a784",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
