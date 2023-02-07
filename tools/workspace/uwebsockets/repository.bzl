# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        commit = "v20.36.0",
        sha256 = "cda266f7ed6abe67ef3cae6e223a580fe5091db9156c1f4123ee328ae21511c9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
