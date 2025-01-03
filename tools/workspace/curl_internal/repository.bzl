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
        commit = "curl-8_11_1",
        sha256 = "78baef817c33a0a11b990d031a706dc1486f614b581e22949fc101add32c4cc8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
