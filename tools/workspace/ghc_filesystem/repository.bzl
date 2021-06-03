# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ghc_filesystem_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gulrak/filesystem",
        commit = "v1.5.6",
        sha256 = "16358d68f7fb1024380bc4619873b8003a5cdaa8700a0bc88ac3c6e96cbc6d48",  # noqa
        build_file = "@drake//tools/workspace/ghc_filesystem:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
