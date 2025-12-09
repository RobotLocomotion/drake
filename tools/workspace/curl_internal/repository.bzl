load("//tools/workspace:github.bzl", "github_archive")

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
        commit = "curl-8_17_0",
        sha256 = "fd680a1b5378294d5c4df4387eec2e6d2a3b4708d3cff7de055db7ab81f8cde5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
