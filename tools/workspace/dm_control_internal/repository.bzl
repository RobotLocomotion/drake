# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "644b3c6844cafd611cb2dfe82bc68bb6aed97292",
        sha256 = "873255140a3ff1d0e9b2275cf66ded00d74be0bfa763dd280312a569e7f4994f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
