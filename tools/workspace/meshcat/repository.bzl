# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "33c1213a26d2e05a3b6ca95c7bcb94689f99e005",
        sha256 = "cf5ea852721cc9e47ee289dcbadf6ddcb1d6b2217ea9cf13d260e757e458d574",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
