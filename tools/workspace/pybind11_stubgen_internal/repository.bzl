# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pybind11_stubgen_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sizmailov/pybind11-stubgen",
        commit = "2536dbc3cad276c9edea86a3af61de9a57ab644d",
        sha256 = "e7350051a4179516cc740745d69a847eecf83dad902e6b23b18cac43dd1c9e9e",  # noqa
        build_file = "@drake//tools/workspace/pybind11_stubgen_internal:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
