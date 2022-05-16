# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def github3_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sigmavirus24/github3.py",
        commit = "3.2.0",
        sha256 = "42cf8e721437a0bcfb05e767302c3221cdc96f3e9db3d76ce990fd0526af1d99",  # noqa
        build_file = "@drake//tools/workspace/github3_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
