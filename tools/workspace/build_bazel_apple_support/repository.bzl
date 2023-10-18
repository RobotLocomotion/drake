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
        commit = "1.10.1",
        sha256 = "0d53d2ba4c3bcf46dd7891649b1a92d49934b29123748d8fd19c6738e8a92415",  # noqa
        mirrors = mirrors,
    )
