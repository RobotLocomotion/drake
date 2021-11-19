# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def github3_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sigmavirus24/github3.py",
        commit = "3.0.0",
        sha256 = "1bdd6f19d2c5756cdb017fb47828d31e77916bba0312b2726c67253a439ae61b",  # noqa
        build_file = "@drake//tools/workspace/github3_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
