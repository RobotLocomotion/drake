# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "3c67a42d021b97808fcc075c0c468d78f0c2233f",
        sha256 = "689a3c01c4370f6d6d4760300b12c4d41d1eee27ba68416eead42db87d64a9f6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
