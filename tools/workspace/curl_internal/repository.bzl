# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def curl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "curl/curl",
        upgrade_advice = """
        In case of a cmake_configure_file build error when upgrading curl,
        update cmakedefines.bzl to match the new upstream definitions.
        """,
        commit = "curl-7_88_1",
        sha256 = "eb9f2ca79e2c39b89827cf2cf21f39181f6a537f50dc1df9c33d705913009ac4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
