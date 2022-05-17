# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def partio_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "wdas/partio",
        commit = "7cb3743c6e19c04ac049c05f8f81af2f24410ea3",
        sha256 = "3cf6a15f9b33e6f606d6363d7c75edf658af802f7d99ff67538fdec0260039dc",  # noqa
        build_file = "@drake//tools/workspace/partio:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
        patches = [
            "@drake//tools/workspace/partio:partio.patch",
        ],
    )
