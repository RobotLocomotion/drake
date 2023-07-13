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
        commit = "gz-math7_7.2.0",
        sha256 = "9de884f77ba048b3651ec9a600457d73e4914637b3bfa5541675ef6774147127",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
