# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def octomap_repository(name):
    github_archive(
        name = name,
        repository = "OctoMap/octomap",
        commit = "v1.8.1",
        sha256 = "8b18ef7693e87f1400b9a8bc41f86e3b28259ac98c0b458037232652380aa6af",  # noqa
        build_file = "@drake//tools/workspace/octomap:package.BUILD.bazel",
    )
