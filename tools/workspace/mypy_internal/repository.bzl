# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/mypy",
        # TODO(mwoehlke-kitware): switch to a tag >= v0.980.
        commit = "3ae19a25f0a39358ede1383e93d44ef9abf165e0",
        sha256 = "866503aed58d7207c0fe4bca9a6a51c1eaa6668157b1c5ed61b40eda71af9175",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
