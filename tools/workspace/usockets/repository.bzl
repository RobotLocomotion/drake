# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "uNetworking/uSockets",
        # NOTE: Do not upgrade without testing the tutorials on Deepnote.  See
        # Drake #18289.  v0.8.5 was tested and showed the same symptoms.
        commit = "v0.8.1",
        sha256 = "3b33b5924a92577854e2326b3e2d393849ec00beb865a1271bf24c0f210cc1d6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
