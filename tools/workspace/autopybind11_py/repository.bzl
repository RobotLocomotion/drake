# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def autopybind11_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "autopybind11/autopybind11",
        commit = "750e03b4149ae022fbcf2d626e82713cf50db539",
        sha256 = "8660dd8de0072d2a9cd564d357c2f5c8661ca67728ec07f3367b3081d374b7a2",  # noqa
        build_file = "@drake//tools/workspace/autopybind11_py:package.BUILD.bazel",  # noqa
        mirrors = {
            "github": ["https://gitlab.kitware.com/{repository}/-/archive/{commit}/autopybind11-{commit}.tar.gz"],  # noqa
        },
    )
