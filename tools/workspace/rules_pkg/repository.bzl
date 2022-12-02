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
        commit = "0.8.0",
        sha256 = "ab55ed03c8e10b5cfd0748a4f9cf5d55c23cb7a7e623c4d306b75684e57483e4",  # noqa
        mirrors = mirrors,
    )
