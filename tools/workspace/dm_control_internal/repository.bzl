# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "41d0c7383153f9ca6c12f8e865ef5e73a98759bd",
        sha256 = "712fbc5b81518b640b1f1c084b22bebc558a8acecec4831ef5f6ca63181431d6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
