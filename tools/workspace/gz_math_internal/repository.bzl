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
        commit = "gz-math9_9.3.0",
        sha256 = "9e957d024a8ce7a94dfb4a33f6425077b33526a0f0e2385ae93c25f88ff8cfb7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
