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
        commit = "curl-8_11_0",
        sha256 = "5a231145114589491fc52da118f9c7ef8abee885d1cb1ced99c7290e9a352f07",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
