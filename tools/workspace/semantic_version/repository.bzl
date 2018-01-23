# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def semantic_version_repository(name):
    pypi_archive(
        name = name,
        package = "semantic_version",
        version = "2.6.0",
        sha256 = "2a4328680073e9b243667b201119772aefc5fc63ae32398d6afafff07c4f54c0",  # noqa
        strip_prefix = "semantic_version",
        build_file = "@drake//tools/workspace/semantic_version:package.BUILD.bazel",  # noqa
    )
