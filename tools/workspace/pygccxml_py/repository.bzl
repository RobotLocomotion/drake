# -*- mode: python; -*-
# vi: set ft=python:

load("@drake//tools/workspace:github.bzl", "github_archive")

def pygccxml_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "CastXML/pygccxml",
        commit = "v2.2.1",
        sha256 = "9815a12e3bf6b83b2e9d8c88335fb3fa0e2b4067d7fbaaed09c3bf26c6206cc7",  # noqa
        build_file = "@drake//tools/workspace/pygccxml_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
