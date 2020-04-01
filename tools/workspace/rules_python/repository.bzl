# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_python",  # License: Apache-2.0
        commit = "748aa53d7701e71101dfd15d800e100f6ff8e5d1",
        sha256 = "64a3c26f95db470c32ad86c924b23a821cd16c3879eed732a7841779a32a60f8",  # noqa
        mirrors = mirrors,
    )
