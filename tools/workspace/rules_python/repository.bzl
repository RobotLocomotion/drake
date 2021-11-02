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
        commit = "0.5.0",
        sha256 = "a2fd4c2a8bcf897b718e5643040b03d9528ac6179f6990774b7c19b2dc6cd96b",  # noqa
        mirrors = mirrors,
    )
