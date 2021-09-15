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
        extra_strip_prefix = "pkg",
        commit = "0.5.1",
        sha256 = "0d2e97f71161e8af97cb2fffe321017293127f6ea8b497cb27b2b8a711e64174",  # noqa
        mirrors = mirrors,
    )
