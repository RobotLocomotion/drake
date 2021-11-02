# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.8.0",
        sha256 = "9116bd3686beaa22be34be1e5259fb9eecbf246a3991849d33ff6ab07d52f86e",  # noqa
        build_file = "@drake//tools/workspace/pycodestyle:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
