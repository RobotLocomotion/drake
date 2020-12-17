# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.6.0",
        sha256 = "08347fbc48cc92afd33117c1e8af9b99b292a4e5889f6b776f402e062fc39c97",  # noqa
        build_file = "@drake//tools/workspace/pycodestyle:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
