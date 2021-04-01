# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ghc_filesystem_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gulrak/filesystem",
        commit = "v1.5.4",
        sha256 = "fc40ec517fa7457a44dc9adabcbe4245951115a08967bb48f9d632dd40d14b4e",  # noqa
        build_file = "@drake//tools/workspace/ghc_filesystem:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
