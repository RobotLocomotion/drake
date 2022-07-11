# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_math_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-math",
        commit = "ignition-math6_6.11.0",
        sha256 = "e6b8901c94147e2c2659323083ce1d151495a07f9bef72a957069ce5b9f3d9e8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
