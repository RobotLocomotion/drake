# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def github3_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sigmavirus24/github3.py",
        commit = "1.3.0",
        sha256 = "c6951691c5e1d182c042b49c9fe88dc73f84309c4393f0a8d5684a02464c35a2",  # noqa
        build_file = "@drake//tools/workspace/github3_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
