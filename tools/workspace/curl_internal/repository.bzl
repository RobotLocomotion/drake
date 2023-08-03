load("@drake//tools/workspace:github.bzl", "github_archive")

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
        commit = "curl-8_2_1",
        sha256 = "230d61a4b1eb3346930f2d601cc8fe5237957163e16befbe15e0ef40c56767a2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
