# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.4.0",
        sha256 = "1843536fb8856a7e2dca94ef9e85504729e095a3a1ff58809583bc13b3dd6e65",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
