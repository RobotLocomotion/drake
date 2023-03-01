# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "774f46182140106e22725914aad3c6299ed91edd",
        sha256 = "69a825c9b984453e3c8e91f383558b83d3f2b0305c5ab7b21d0772cb1da9c659",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
