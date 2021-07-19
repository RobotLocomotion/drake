# -*- mode: python; -*-
# vi: set ft=python:

load("@drake//tools/workspace:github.bzl", "github_archive")

def autopybind11_py_repository(name):
    github_archive(
        name = name,
        repository = "autopybind11/autopybind11",
        commit = "9cb9754f7d8197cefe68f093ac52b8756f32871e",
        sha256 = "290e8126b3b84cff961b15a18b520386feadda3f756fac476784eb577761f91f",  # noqa
        build_file = "@drake//tools/workspace/autopybind11_py:package.BUILD.bazel",  # noqa
        mirrors = {
            "github": [
                "https://gitlab.kitware.com/{repository}/-/archive/{commit}/autopybind11-{commit}.tar.gz",  # noqa
                "https://drake-mirror.csail.mit.edu/gitlab.kitware.com/{repository}/autopybind11-{commit}.tar.gz",  # noqa
                "https://s3.amazonaws.com/drake-mirror/gitlab.kitware.com/{repository}/autopybind11-{commit}.tar.gz",  # noqa
            ],
        },
    )
