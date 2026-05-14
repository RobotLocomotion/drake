load("//tools/workspace:github.bzl", "github_archive")

def uwebsockets_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "uNetworking/uWebSockets",
        commit = "v20.77.0",
        sha256 = "9884f7bce0d34c776d450f1a1f31858aaced74c4454f2758543ed557df07d1e0",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/no_globals.patch",
            ":patches/max_fallback_size.patch",
        ],
        mirrors = mirrors,
    )
