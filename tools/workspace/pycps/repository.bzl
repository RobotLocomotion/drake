# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "bd482e8c31fd6636105e975870267c2654f6af73",
        sha256 = "26b589fd8fdcd1858ca95cdc554d833f12c5311a4e249bac56f596faf0d3a8b2",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
