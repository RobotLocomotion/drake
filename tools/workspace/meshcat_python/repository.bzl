# -*- mode: python -*-
# vi: set ft=python :

"""
Makes the meshcat module from meshcat-python available to be used as a Python
dependency. A meshcat-server console script is also created. On all platforms,
a meshcat-python archive is downloaded from GitHub (https://git.io/fxbL0) or a
specified mirror and unpacked.

Example:
    WORKSPACE:
        load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
        load(
            "@drake//tools/workspace/meshcat_python:repository.bzl",
            "meshcat_python_repository",
        )
        meshcat_python_repository(name = "foo", mirrors = DEFAULT_MIRRORS)

    BUILD:
        py_library(
            name = "foobar",
            deps = ["@foo//:meshcat"],
            srcs = ["bar.py"],
        )

    Command Line:
        $ bazel run @foo//:meshcat-server

Arguments:
    name: A unique name for this rule. The rule must not be named meshcat.
    mirrors: A dictionary of mirrors, see tools/workspace/mirrors.bzl for an
             example.
"""

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_python_repository(
        name,
        mirrors = None):
    if name == "meshcat":
        fail("Rule must NOT be named meshcat")

    github_archive(
        name = name,
        repository = "rdeits/meshcat-python",
        commit = "v0.0.13",
        sha256 = "e163a9bd55221ebaecbe15946481700e4c7dfbb9e231fa2bd25b852f9dcf1c6f",  # noqa
        build_file = "@drake//tools/workspace/meshcat_python:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
