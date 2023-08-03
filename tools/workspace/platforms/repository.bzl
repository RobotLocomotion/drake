load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def platforms_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/platforms",  # License: Apache-2.0
        commit = "0.0.7",
        sha256 = "dd1e88b70f645533ea59f418b501120af71ef107edeaa4d12e2a4ec75b59924c",  # noqa
        mirrors = mirrors,
    )
