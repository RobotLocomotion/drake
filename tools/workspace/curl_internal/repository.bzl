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
        commit = "curl-8_19_0",
        sha256 = "3342390f774bda7523464a13c535312962a27b026e7a05ec8dab027a8480327e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
