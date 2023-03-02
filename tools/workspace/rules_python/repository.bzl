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
        repository = "bazelbuild/rules_python",  # License: Apache-2.0,
        upgrade_advice = """
        The commit (version) and sha256 here should be identical to the
        commit listed in
        drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        """,
        commit = "0.17.3",
        sha256 = "8c15896f6686beb5c631a4459a3aa8392daccaab805ea899c9d14215074b60ef",  # noqa
        mirrors = mirrors,
    )
