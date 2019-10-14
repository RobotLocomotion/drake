# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.19.10.3",
        sha256 = "ebababd6db551014c97933d909a6822ee8f65959be8eb32c8c285c7692d3617e",  # noqa
        mirrors = mirrors,
    )
