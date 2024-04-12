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
        commit = "gz-math7_7.4.0",
        sha256 = "e7c7d09cba113f437fc6a8f39729ebd273c79a3779195881e6dcf0c45803a410",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
