# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def github3_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sigmavirus24/github3.py",
        commit = "3.1.2",
        sha256 = "32eb15b53459dd7fed9856402d6a4e67d1ad2dddc22928838fb44eb7ca9eca9c",  # noqa
        build_file = "@drake//tools/workspace/github3_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
