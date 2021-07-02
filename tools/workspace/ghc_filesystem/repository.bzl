# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ghc_filesystem_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gulrak/filesystem",
        commit = "v1.5.8",
        sha256 = "726f8ccb2ec844f4c66cc4b572369497327df31b86c04cefad6b311964107139",  # noqa
        build_file = "@drake//tools/workspace/ghc_filesystem:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
