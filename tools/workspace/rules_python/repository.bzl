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
        # The commit (version) and sha256 here should be identical to the
        # commit listed in
        # drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        commit = "0.8.0",
        sha256 = "9fcf91dbcc31fde6d1edb15f117246d912c33c36f44cf681976bd886538deba6",  # noqa
        mirrors = mirrors,
    )
