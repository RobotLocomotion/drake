# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.5.0",
        sha256 = "c8fd5561e1bd88b743ef2ee065a5e661b2fd7b56e9cbe9ae2aeb928f41438819",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
