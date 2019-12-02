# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "9fa020fd110a09be44dcb4522b9ce55ef671f34c",
        sha256 = "561f04736e98d704a9c8246dd84aedac7ea234d6710a420eca4c61be482c09e2",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
