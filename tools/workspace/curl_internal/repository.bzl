# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def curl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "curl/curl",
        # In case of a cmake_configure_file build error when upgrading curl,
        # update cmakedefines.bzl to match the new upstream definitions.
        commit = "curl-7_87_0",
        sha256 = "41cf1305f8f802616a27474ecf68135faeac38beccbe4e5f15a8c7e6f05a15a9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
