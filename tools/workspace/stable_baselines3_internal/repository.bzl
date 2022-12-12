# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v1.6.0",
        sha256 = "f6642fb002adf7ce10087319ea8e9a331d95d26f6558067339f26c84fc588bb6",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":connection.patch",
        ],
        mirrors = mirrors,
    )
