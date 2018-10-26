# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        # PR DRAFT: DO NOT MERGE
        commit = "688c21eb77ddc2b8afe471db0bc92ce7831ed6af",
        sha256 = "b7164c68f65e76a6273e5d5a57f277da99de57488784d1b03b7f20692334de2d",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
