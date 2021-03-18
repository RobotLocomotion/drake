# -*- python -*-

load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def toposort_py_repository(
        name,
        mirrors = None):
    pypi_archive(
        name = name,
        package = "toposort",
        version = "1.5",
        sha256 = "dba5ae845296e3bf37b042c640870ffebcdeb8cd4df45adaa01d8c5476c557dd",  # noqa
        build_file = "@drake//tools/workspace/toposort_py:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
