# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.29.0",
        sha256 = "965f7969be61b80fb004c85c2a205b1277e8bfd09c3b640bd40f3ff04bc33c66",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/issue1512.patch",
        ],
        mirrors = mirrors,
    )
