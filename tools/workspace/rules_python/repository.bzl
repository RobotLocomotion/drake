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
        commit = "0.0.3",
        sha256 = "3720faae8aed1415c3af58d241ecffccfa4d31f2bb70ffa2f6dfb68ca9729626",  # noqa
        mirrors = mirrors,
    )
