# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def semantic_version_repository(
        name,
        mirrors = None):
    pypi_archive(
        name = name,
        package = "semantic_version",
        version = "2.8.4",
        sha256 = "352459f640f3db86551d8054d1288608b29a96e880c7746f0a59c92879d412a3",  # noqa
        strip_prefix = "semantic_version",
        build_file = "@drake//tools/workspace/semantic_version:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
