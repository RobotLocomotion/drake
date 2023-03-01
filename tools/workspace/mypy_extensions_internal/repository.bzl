# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "python/mypy_extensions",
        commit = "1.0.0",
        sha256 = "c1f1fc0cc5f5be7d3a70b6dd4b85f9e2b02d788d66f3a168652a65df6571df07",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
