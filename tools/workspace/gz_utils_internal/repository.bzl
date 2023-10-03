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
        commit = "gz-utils2_2.1.0",
        sha256 = "667d5000ff02ea332e6d4aa9fba1e31b2d3d153e69d40693918a8bb44330ca57",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
