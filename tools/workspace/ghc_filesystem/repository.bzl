# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ghc_filesystem_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gulrak/filesystem",
        commit = "v1.5.10",
        sha256 = "9b96a024679807879fdfb30e46e8e461293666aeeee5fbf7f5af75aeacdfea29",  # noqa
        build_file = "@drake//tools/workspace/ghc_filesystem:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
