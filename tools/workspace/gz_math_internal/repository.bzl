# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_math_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/gz-math",
        commit = "gz-math7_7.1.0",
        sha256 = "3df09a16b84fa27fabf4955b5efb207f417ef9b0b5b801ae28cfda6d8e11765a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
