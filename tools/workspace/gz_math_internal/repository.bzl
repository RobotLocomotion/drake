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
        commit = "gz-math7_7.3.0",
        sha256 = "320afb30f43727340718743c0f8d94e61e9c1083c17927701efe75087f9cc6d0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
