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
        commit = "gz-math7_7.5.1",
        sha256 = "9506f0940045f12d6dee5eef0df36737a580451e3577c003fe347aab09d8702b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
