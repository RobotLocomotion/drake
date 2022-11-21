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
        commit = "curl-7_86_0",
        sha256 = "8085bfc250ffa6f3b5b1b05bea42b368a0361ede004f7c2b9edf102a59e8efb4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
