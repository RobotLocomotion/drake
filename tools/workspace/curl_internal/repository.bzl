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
        commit = "curl-8_10_1",
        sha256 = "5aaf131294f734756325dd99d849518c9a5060fc702517ab3064c76257dc700c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
