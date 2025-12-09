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
        commit = "v20.74.0",
        sha256 = "e1d9c99b8e87e78a9aaa89ca3ebaa450ef0ba22304d24978bb108777db73676c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
