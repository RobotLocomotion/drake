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
        commit = "curl-8_3_0",
        sha256 = "8a56cf0cd80788ecd7f3914a55a35f772d2d3956961c75c6a15419901c452409",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
