# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/mypy",
        commit = "f85dfa1b2533621094bc45b4263ea41fd3bc2e39",
        sha256 = "68800395a9fbaba5494710d0a316eb1c7090cc5553ab53eb62219939e58a67fd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
