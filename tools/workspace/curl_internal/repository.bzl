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
        commit = "curl-8_7_1",
        sha256 = "0e46c856f517602c347bb5fe5b73174f8ee798bc87f1a97235c95761f75fcc28",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
