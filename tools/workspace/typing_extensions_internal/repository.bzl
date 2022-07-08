# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.3.0",
        sha256 = "9dbc928aed2839a23d210726697700a1c4593ab3bbf82b981fcc44585a47ce30",  # noqa
        build_file = "@drake//tools/workspace/typing_extensions_internal:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
