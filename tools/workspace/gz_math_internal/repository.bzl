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
        upgrade_type = "tag",
        tags_pattern = "^(gz)",
        commit = "gz-math9_9.2.0",
        sha256 = "fed32da2ac16b96c45e87ceb03b0fae6cbc8a860c8341bd9a2549dffee1c1f59",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
