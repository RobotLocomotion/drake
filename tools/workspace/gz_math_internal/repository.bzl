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
        commit = "gz-math9_9.0.0",
        sha256 = "e1eacfc2bf6b875ec270b6323cfaa8dfe043c9aaff9ceb8f87ff46e1cde9474a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
