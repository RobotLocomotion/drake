# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.30.0",
        sha256 = "00a6b1aa8e980254d392fe6472adf4fbd551d66412cf8fd5323c762ad1861021",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
