# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.7.0",
        sha256 = "5651c4b981bb0620e6ffb6a94dd6a68ec7fdcbd88c3219e5a976424692700f56",  # noqa
        build_file = "@drake//tools/workspace/pycodestyle:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
