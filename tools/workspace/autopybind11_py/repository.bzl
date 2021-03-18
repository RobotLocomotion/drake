# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def autopybind11_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "autopybind11/autopybind11",
        commit = "6d03a79aea57f281f5625c10e0bb1c0dff734f0f",
        sha256 = "4c8d7e9d13d60f12d98464c711fb87144ed8fd9d44462c91f1ad2bb765ffa499",  # noqa
        build_file = "@drake//tools/workspace/autopybind11_py:package.BUILD.bazel",  # noqa
        mirrors = {
            "github": ["https://gitlab.kitware.com/{repository}/-/archive/{commit}/autopybind11-{commit}.tar.gz"],  # noqa
        },
    )
