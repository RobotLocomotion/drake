load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "yugr/Implib.so",
        commit = "ae294daac027700835e1fc17e675b4a21bf7d76d",
        sha256 = "5daeeb662bf5b56b8a35ab581236567480d8e284cf3193c1f9a5a9f037cfe5f9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
