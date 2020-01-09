# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uritemplate_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python-hyper/uritemplate",
        commit = "3.0.1",
        sha256 = "0dbac32598f33bcbbf871a06285c81e175e061284403792b054d1254486c609d",  # noqa
        build_file = "@drake//tools/workspace/uritemplate_py:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
