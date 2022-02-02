# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        # TODO(BetsyMcPhail) v20.11.0 requires a header not available on
        # the GCC7 used on Ubuntu 18.04 (Bionic). Do not upgrade until
        # Bionic support is dropped.
        commit = "v20.9.0",
        sha256 = "91897568291a2081ffc1ae27c354446cc130c168a25892b2289f25e185af8676",  # noqa
        build_file = "@drake//tools/workspace/uwebsockets:package.BUILD.bazel",
        mirrors = mirrors,
    )
