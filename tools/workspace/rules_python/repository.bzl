# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_python",  # License: Apache-2.0
        commit = "0.0.1",
        sha256 = "fa53cc0afe276d8f6675df1a424592e00e4f37b2a497e48399123233902e2e76",  # noqa
        mirrors = mirrors,
    )
