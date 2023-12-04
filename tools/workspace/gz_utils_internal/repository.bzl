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
        commit = "gz-utils2_2.2.0",
        sha256 = "15846369999e1269ab4dcb2f9fd2b4acdd162a69ae40a3f1cd3889437173d3aa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
