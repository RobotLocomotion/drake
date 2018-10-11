# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "d14257769e0110f542c7d3a0f6b0d2c80d59ff52",
        sha256 = "3f36d5f2a7084749d4e602197c428816a5f28f39c13b6e01de78d8999bfc470e",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
