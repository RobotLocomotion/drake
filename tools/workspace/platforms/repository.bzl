load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def platforms_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/platforms",  # License: Apache-2.0
        commit = "0.0.9",
        sha256 = "b929b3159b8d8f3c8a4675a23211b0f36607352e97ed8e684d3d3e3b9d4f2e13",  # noqa
        mirrors = mirrors,
    )
