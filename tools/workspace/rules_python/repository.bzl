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
        commit = "0.0.2",
        sha256 = "a1b2797d85abb395468915f2ab652cee65e9a44e31bff559cd7de0dedf056363",  # noqa
        mirrors = mirrors,
    )
