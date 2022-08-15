# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.9.0",
        sha256 = "de1438a604f234065f446f45069ef908c007084b1d3abe2e2f89c98ab6fb9d25",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
