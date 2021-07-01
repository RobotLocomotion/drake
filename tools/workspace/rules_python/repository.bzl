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
        commit = "0.3.0",
        sha256 = "4feecd37ec6e9941a455a19e7392bed65003eab0aa6ea347ca431bce2640e530",  # noqa
        mirrors = mirrors,
    )
