# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.5.0",
        sha256 = "a603453c07e8d8e15a43cf062aa7174741b74b4a27b110f9ad03d74d519173b5",  # noqa
        build_file = "@drake//tools/workspace/pycodestyle:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
