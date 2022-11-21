# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.9.1",
        sha256 = "d6d8182c2fe10f169192b1133cb11c008ca712da01ce41d8c14523f644c6fe05",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
