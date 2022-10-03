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
        commit = "0.7.1",
        sha256 = "d258fb6965cf3d7ebdbe146ec7e28b605f0644cb880101604e166e35d4ca62bc",  # noqa
        mirrors = mirrors,
    )
