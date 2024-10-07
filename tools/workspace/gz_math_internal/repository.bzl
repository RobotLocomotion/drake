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
        commit = "gz-math8_8.0.0",
        sha256 = "a78962f329eea0dd70268ca1c196bc28729f1857ec7cd79cacc0ab1269f55b79",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
