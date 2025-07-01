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
        commit = "gz-math8_8.2.0",
        sha256 = "6a6e9af99378b4ab81392bd8ad58be1c611f917c669c981b39cf20fc03d412f4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
