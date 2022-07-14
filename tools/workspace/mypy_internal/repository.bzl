# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/mypy",
        # TODO(mwoehlke-kitware): switch to a tag >= v0.980.
        commit = "7c6faf4c7a7bac6b126a30c5c61f5d209a4312c0",
        sha256 = "f0ab4a26f0f75fc345865b17e4c21f34f13b7d9220eab663e96116dd326b9e48",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
