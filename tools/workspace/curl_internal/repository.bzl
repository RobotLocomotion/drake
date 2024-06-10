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
        commit = "curl-8_8_0",
        sha256 = "eb2f17efd68013945ffde1cb0ca61bf46709db86f400c76e84463aef453ed74e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
