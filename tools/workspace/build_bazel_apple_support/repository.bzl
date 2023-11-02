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
        commit = "1.11.1",
        sha256 = "1f316618f49501f37749a402a4ce41c8431ca8063d57e913edf72a69638a3344",  # noqa
        mirrors = mirrors,
    )
