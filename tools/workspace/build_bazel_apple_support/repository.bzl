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
        commit = "1.8.1",
        sha256 = "826ec7a72cb057bfc9c351dcc0200bf475613c5ccc7a14d653d725f5ab8604bc",  # noqa
        mirrors = mirrors,
    )
