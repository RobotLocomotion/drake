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
        commit = "curl-8_9_1",
        sha256 = "d714818f6ac41ae9154850158fed44b7a87650a6d52f83d3bcb9aa527be354d7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
