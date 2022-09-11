# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_utils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-utils",
        commit = "gz-utils2_2.0.0-pre1",
        sha256 = "b3a6236f9d38fefcff19509e69a596f42c82aa76354dd91aa974a8c76be4a053",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
