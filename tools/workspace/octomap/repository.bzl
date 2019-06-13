# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def octomap_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "OctoMap/octomap",
        commit = "v1.9.0",
        sha256 = "5f81c9a8cbc9526b2e725251cd3a829e5222a28201b394314002146d8b9214dd",  # noqa
        build_file = "@drake//tools/workspace/octomap:package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            "@drake//tools/workspace/octomap:do_not_use_random_shuffle.patch",
        ],
    )
