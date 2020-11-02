# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_python",  # License: Apache-2.0
        commit = "0.1.0",
        sha256 = "48f7e716f4098b85296ad93f5a133baf712968c13fbc2fdf3a6136158fe86eac",  # noqa
        mirrors = mirrors,
    )
