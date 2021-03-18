# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def configargparse_py_repository(
        name,
        mirrors = None):
    pypi_archive(
        name = name,
        package = "ConfigArgParse",
        version = "1.2.3",
        sha256 = "edd17be986d5c1ba2e307150b8e5f5107aba125f3574dddd02c85d5cdcfd37dc",  # noqa
        build_file = "@drake//tools/workspace/configargparse_py:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
