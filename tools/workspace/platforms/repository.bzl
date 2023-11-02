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
        commit = "0.0.8",
        sha256 = "58ca5559d562def65cf1aeae9cd994d2776f7273eab9f48779ad043c3ffb3ce3",  # noqa
        mirrors = mirrors,
    )
