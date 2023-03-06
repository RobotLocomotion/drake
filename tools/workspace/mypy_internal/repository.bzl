# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def mypy_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "python/mypy",
        commit = "v1.0.1",
        sha256 = "44e971ab71dc8d06fee3c72f03d80cf0ee39d2dd3a65da1ecdd8007861dc17c4",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
