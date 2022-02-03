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
        commit = "0.6.0",
        sha256 = "04535dbfbdf3ec839a2c578a0705a34e5a0bbfd4438b29e285b961e6e0b97ce1",  # noqa
        mirrors = mirrors,
    )
