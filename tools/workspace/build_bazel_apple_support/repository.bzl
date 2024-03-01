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
        commit = "1.14.0",
        sha256 = "c62323d024eb512060714963051bbbfce007e652bc76fd68183504dd1585c119",  # noqa
        mirrors = mirrors,
    )
