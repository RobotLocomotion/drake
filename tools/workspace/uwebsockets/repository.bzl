# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "uNetworking/uWebSockets",
        # NOTE: Do not upgrade without testing the tutorials on Deepnote.  See
        # Drake #18289.  v20.35.0 was tested and showed the same symptoms.
        commit = "v20.14.0",
        sha256 = "15cf995844a930c9a36747e8d714b94ff886b6814b5d4e3b3ee176f05681cccc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
