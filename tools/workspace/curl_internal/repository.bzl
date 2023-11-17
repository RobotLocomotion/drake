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
        commit = "curl-8_4_0",
        sha256 = "5f7097e27b00e54d5522d80c0d44899084a226e9372a0d5d799ad3125dd4b8c0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
