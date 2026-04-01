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
        commit = "gz-math9_9.1.0",
        sha256 = "d6d266a2a5094b977a3cfec4646efb2eede5fd36781a53faaa37ba416da5cdaf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
