# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "a91a21e1421a475f8db2ab60d4eaf14d29c305b1",
        sha256 = "6bb8afb5b7229fb8dad73634c9c9b2659f8a9daa2043957683bf4a379f03b550",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
