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
        commit = "1.17.0",
        sha256 = "8221df51df40ce8d3a1f74935d7911dfbfd074c91d17e67d8e5b9f59a4a6a511",  # noqa
        patches = [
            ":patches/no_bazel_features.patch",
        ],
        mirrors = mirrors,
    )
