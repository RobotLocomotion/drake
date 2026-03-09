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
        commit = "v20.75.0",
        sha256 = "05c384e410d374ce81abc17c9ee9574f33611d4a020f1da26f444f0dfe9691d7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
