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
        commit = "curl-7_85_0",
        sha256 = "178331707b434719dcad0e446e6ae4ac4c4b2046e9387baed74c5e2ba4cd4a45",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
