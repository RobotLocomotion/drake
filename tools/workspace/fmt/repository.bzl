# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fmt_repository(name):
    github_archive(
        name = name,
        repository = "fmtlib/fmt",
        commit = "3.0.1",
        sha256 = "dce62ab75a161dd4353a98364feb166d35e7eea382169d59d9ce842c49c55bad",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
    )
