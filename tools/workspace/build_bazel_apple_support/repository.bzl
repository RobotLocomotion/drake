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
        commit = "1.15.1",
        sha256 = "d05f4352f46ea182b077b05c18d57d39daf8bf6e8aa25e7f5f4a893ecabe98de",  # noqa
        mirrors = mirrors,
    )
