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
        commit = "curl-8_16_0",
        sha256 = "d4d9a5001b491f5726efe9b50bc4aad03029506e73c9261272e809c64b05e814",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
