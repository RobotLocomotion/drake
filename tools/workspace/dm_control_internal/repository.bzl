# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "f69785aeafaf891601236e2a0ec0c6e38756604f",
        sha256 = "d059c285ba0f364c00de3dbaef5c2515cf26b735258aad8fb3a91202d304684a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
