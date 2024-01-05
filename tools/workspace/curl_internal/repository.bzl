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
        commit = "curl-8_5_0",
        sha256 = "8117d24a8c29a0c3aa160703eb487694f3d4aa72ea2067b45beb439ea4d47668",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
