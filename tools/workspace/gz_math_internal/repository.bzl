# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_math_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-math",
        commit = "gz-math7_7.0.2",
        sha256 = "3b3db8b5d95d2b3de72d57956aa727bfb78c40c1776a54a1badf3ba9c2cd33a1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
