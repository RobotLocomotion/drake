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
        upgrade_type = "release",
        commit = "curl-8_23_0",
        sha256 = "d0605d5883124ef5ae64c6d2b307e1417c482b362583b57491fdf37d9a99a792",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
