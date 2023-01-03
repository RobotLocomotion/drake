# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gym_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "openai/gym",
        commit = "0.26.2",
        sha256 = "96a5fc8345bd92b73a15fc25112d53a294f86fcace1c5e4ef7f0e052b5e1bdf4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
