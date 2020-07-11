# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pygccxml_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "CastXML/pygccxml",
        # Need bleeding edge from `develop` for C++17 support.
        commit = "a2ae7a72891246207285d5e15dd25dec14412296",
        sha256 = "bd8df585118b1f073bd2634f5b4f8def77648e702b4ab51bb8aca94715388c5c",  # noqa
        build_file = "@drake//tools/workspace/pygccxml_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
