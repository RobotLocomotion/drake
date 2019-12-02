# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def octomap_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "OctoMap/octomap",
        commit = "v1.9.1",
        sha256 = "9abce615d9f3f97a15ba129a10e3a01f9bef9aad178f2ef398f9a925f793c7b9",  # noqa
        build_file = "@drake//tools/workspace/octomap:package.BUILD.bazel",
        mirrors = mirrors,
    )
