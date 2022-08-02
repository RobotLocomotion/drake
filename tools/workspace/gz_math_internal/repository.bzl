# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_math_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-math",
        commit = "ignition-math6_6.12.0",
        sha256 = "4a5da88a45da7763d4551866d0007d270a8ab5ae5bf67ea3f891fc6799c6d84f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
