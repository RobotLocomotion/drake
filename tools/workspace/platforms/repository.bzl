# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def platforms_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/platforms",  # License: Apache-2.0
        commit = "0.0.6",
        sha256 = "1626b708a06989c2365f3101c9c937153e03ee39faaaeab98a2c204e9d015a0d",  # noqa
        mirrors = mirrors,
    )
