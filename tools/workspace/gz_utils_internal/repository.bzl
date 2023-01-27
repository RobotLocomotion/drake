# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_utils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/gz-utils",
        commit = "gz-utils2_2.0.0",
        sha256 = "af9e5b862e10aa0cedd97d9c5ca3eb9a443b7c9e560a083e8f0399e93e1cfafa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
