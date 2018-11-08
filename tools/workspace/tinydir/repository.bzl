# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinydir_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cxong/tinydir",
        commit = "1.2.4",
        sha256 = "9c50eda69ba4854bb76ffa18961f49fd75f323b4cbebdf6b4b2d2db28f9f5ce2",  # noqa
        build_file = "@drake//tools/workspace/tinydir:package.BUILD.bazel",
        mirrors = mirrors,
    )
