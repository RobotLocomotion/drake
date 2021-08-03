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
        commit = "0.5.0",
        sha256 = "098d9d442a2dbd77bf0329d6c995a36d4f492f3161e80197abee5a9bf7ae262f",  # noqa
        mirrors = mirrors,
    )
