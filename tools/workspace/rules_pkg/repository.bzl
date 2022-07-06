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
        commit = "0.7.0",
        sha256 = "e110311d898c1ff35f39829ae3ec56e39c0ef92eb44de74418982a114f51e132",  # noqa
        mirrors = mirrors,
    )
