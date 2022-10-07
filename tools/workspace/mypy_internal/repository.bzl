# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/mypy",
        commit = "v0.982",
        sha256 = "22d642f7d2e0cf073585e61209d9775f6fa1bf57a2de0a9f992a73a59486d6e1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
