# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_utils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-utils",
        commit = "ignition-utils1_1.4.0",
        sha256 = "74ba40f8a35f9ae07102402d4acde11e4f9a1d52663a339388fe086527093f86",  # noqa
        build_file = "@drake//tools/workspace/gz_utils_internal:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
