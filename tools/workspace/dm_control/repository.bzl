# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "fe4449606742a7b8bec81930790b98244cddc538",
        sha256 = "f1c8a58f1a30f4d8e414c1cf6e18426bf7b2df61cae3dea127f2884033d502eb",  # noqa
        build_file = "@drake//tools/workspace/dm_control:package.BUILD.bazel",
        mirrors = mirrors,
    )
