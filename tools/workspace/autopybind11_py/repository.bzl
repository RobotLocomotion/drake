# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def autopybind11_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "autopybind11/autopybind11",
        commit = "7ead655ed3a94035816f6560a9f2e60983f0c698",
        sha256 = "993936cd8f200ba43b28a3ebecfe045170a3f05fe5382cd85e91c00a04908863",  # noqa
        build_file = "@drake//tools/workspace/autopybind11_py:package.BUILD.bazel",  # noqa
        mirrors = {
            "github": ["https://gitlab.kitware.com/{repository}/-/archive/{commit}/autopybind11-{commit}.tar.gz"],  # noqa
        },
    )
