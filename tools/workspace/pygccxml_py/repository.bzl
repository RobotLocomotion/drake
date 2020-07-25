# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pygccxml_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "CastXML/pygccxml",
        # Need bleeding edge from `develop` for C++17 support.
        commit = "4b3e42a004d6a3a385d82490aa7d9d243df110a1",
        sha256 = "3cb75b4cf1e9b0620c24582e63ef2c080aaaec63eddeb641b395dd0cb6e22439",  # noqa
        build_file = "@drake//tools/workspace/pygccxml_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
