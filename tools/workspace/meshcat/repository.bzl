# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "efc157436ca4abf5706cf248c51d2e86cae1a0f0",
        sha256 = "7fd3c3f86f867135bf0a77a03257302e1751139b1199683418b94fca71e7ff18",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
