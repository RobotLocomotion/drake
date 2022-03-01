# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "4cf89ef1a0d99f7e9f70b3f5ad0bf2a685cbd935",
        sha256 = "4dfbdce85a2bc96add7afa8d3c791b343dfc76c31f50e6fb1b42fd0129b8642e",  # noqa
        build_file = "@drake//tools/workspace/dm_control:package.BUILD.bazel",
        mirrors = mirrors,
    )
