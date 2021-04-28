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
        commit = "0.2.0",
        sha256 = "0d25ab1c7b18b3f48d1bff97bfa70c1625438b40c5f661946fb43eca4ba9d9dd",  # noqa
        mirrors = mirrors,
    )
