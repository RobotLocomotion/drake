load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def build_bazel_apple_support_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/apple_support",  # License: Apache-2.0
        commit = "1.16.0",
        sha256 = "0ac024fa6227658524feb41a271a0671905f1c1208cce97b198ed8fa15964166",  # noqa
        mirrors = mirrors,
    )
