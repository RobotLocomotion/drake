# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def semantic_version_repository(
        name,
        mirrors = None):
    pypi_archive(
        name = name,
        package = "semantic_version",
        version = "2.8.3",
        sha256 = "9dcc6fbad58da3c4d5eee2287025e226bb05c39463f14b741357801baae9dcce",  # noqa
        strip_prefix = "semantic_version",
        build_file = "@drake//tools/workspace/semantic_version:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
