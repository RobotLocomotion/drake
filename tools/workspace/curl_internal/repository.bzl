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
        commit = "curl-8_14_1",
        sha256 = "6cf947ec831e8522b30d7fa8784ce5fcdf1f21581111861d82085a2729c59ba9",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/base64_definition.patch",
        ],
        mirrors = mirrors,
    )
