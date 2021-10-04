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
        # The commit (version) here should be identical to the commit listed in
        # drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        commit = "0.4.0",
        sha256 = "45f22030b4c3475d5beb74ee9a9b86df6e83d5e18c6f23c7ec1a43cea7a31b93",  # noqa
        mirrors = mirrors,
    )
