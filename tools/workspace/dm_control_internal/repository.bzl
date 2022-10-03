# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "f755f6851890da91217094497d55728ff931c640",
        sha256 = "fd7cc1d3e0a01c6cace2f161417902ffb8c97cdd57a55aaa997e984d5182e351",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
