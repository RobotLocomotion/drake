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
        commit = "gz-math8_8.1.0",
        sha256 = "0f27c86008b7d23a0d98e58d1987801ad2ba05e99998d0ad143ad399ee47e745",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/pr656.patch",
        ],
        mirrors = mirrors,
    )
