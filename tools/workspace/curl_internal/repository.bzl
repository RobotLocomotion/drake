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
        commit = "curl-8_12_1",
        sha256 = "6edc063d1ebaf9cf3b3b46e9fef2f3cd8932694989ecd43d005d6e828426d09f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
