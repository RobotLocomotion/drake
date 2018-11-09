# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "c7dcbca3af0e76f3c22573076a8d8e54b52dff67",
        sha256 = "4f2cd2e6cac98e1ffb8b60f13d2a8b15638bf38340e4f8f02d5101cb2babe226",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
