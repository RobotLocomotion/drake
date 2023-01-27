# -*- python -*-

# This dependency is part of a "cohort" defined in
# drake/tools/workspace/new_release.py.  This dependency should only be
# updated in conjunction with the other members of its cohort.

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/mypy_extensions",
        commit = "0.4.3",
        sha256 = "21e830f4baf996d0a9bd7048a5b23722dbd3b88354b8bf0e22376f9a8d508c16",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
