# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that for rules_pkg, we do NOT install its LICENSE file as part of the
# Drake install, because rules_pkg is a build-time tool only.

def rules_pkg_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_pkg",  # License: Apache-2.0
        commit = "0.8.1",
        sha256 = "99d56f7cba0854dd1db96cf245fd52157cef58808c8015e96994518d28e3c7ab",  # noqa
        mirrors = mirrors,
    )
