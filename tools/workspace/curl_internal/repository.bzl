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
        commit = "curl-7_84_0",
        sha256 = "e2e9ef1b31d364e5c0c6cd6cd1b2a6ee14ad9439845ebc67e4535391a3e38e20",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
