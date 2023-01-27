# -*- python -*-

# This dependency is part of a "cohort" defined in
# drake/tools/workspace/new_release.py.  This dependency should only be
# updated in conjunction with the other members of its cohort.

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.35.0",
        sha256 = "907431acec1d480f6e5d29d8c8cc7ace1c96b31e45aebaa18a7e9512fb2f17fc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
