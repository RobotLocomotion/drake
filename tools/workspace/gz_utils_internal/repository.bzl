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
        commit = "gz-utils3_3.1.0",
        sha256 = "f7af72b1524f4192e7b7d7aea50ab2ba27b31ef46f4e6be4f6be5a9b0a2bbf21",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
