# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def semantic_version_repository(
        name,
        mirrors = None):
    pypi_archive(
        name = name,
        package = "semantic_version",
        version = "2.8.5",
        sha256 = "d2cb2de0558762934679b9a104e82eca7af448c9f4974d1f3eeccff651df8a54",  # noqa
        strip_prefix = "semantic_version",
        build_file = "@drake//tools/workspace/semantic_version:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
