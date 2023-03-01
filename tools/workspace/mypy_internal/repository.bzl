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
        commit = "v1.0.0",
        sha256 = "26c144b9a01c0dc244ea4273cdabe7a695e302d5ba6423f2e81c4968b1f0c062",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/reject_double_colon.patch",
            ":patches/timeout.patch",
        ],
        mirrors = mirrors,
    )
