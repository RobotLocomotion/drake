# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def semantic_version_repository(
        name,
        mirrors = None):
    pypi_archive(
        name = name,
        package = "semantic_version",
        version = "2.8.2",
        sha256 = "71c716e99086c44d068262b86e4775aa6db7fabee0743e4e33b00fbf6f672585",  # noqa
        strip_prefix = "semantic_version",
        build_file = "@drake//tools/workspace/semantic_version:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
