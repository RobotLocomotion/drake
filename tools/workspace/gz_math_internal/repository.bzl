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
        commit = "gz-math8_8.1.1",
        sha256 = "0455b24069c61172b625c55348f3a722652058262422019176d0ff43551f7093",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
