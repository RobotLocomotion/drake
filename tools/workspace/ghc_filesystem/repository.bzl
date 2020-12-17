# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ghc_filesystem_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gulrak/filesystem",
        commit = "v1.3.8",
        sha256 = "d8fb4518498a9107aa18af2dfe9a12c5a8031d63e3c1a551847b1826cba4ff08",  # noqa
        build_file = "@drake//tools/workspace/ghc_filesystem:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
