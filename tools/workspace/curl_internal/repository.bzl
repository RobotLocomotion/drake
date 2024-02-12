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
        commit = "curl-8_6_0",
        sha256 = "95d94af73fe84e6ea26480035865c83763dc54911fd4d99b0eb52bb8d165e1a6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
