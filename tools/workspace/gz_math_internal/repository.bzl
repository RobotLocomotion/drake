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
        commit = "gz-math7_7.5.0",
        sha256 = "b323b6082902c7970938f81e7bf515ab035f5bcb9f1b08ea1a5acf03ab247000",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
