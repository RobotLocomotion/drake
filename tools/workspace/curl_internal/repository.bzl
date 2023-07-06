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
        commit = "curl-8_1_2",
        sha256 = "670d6aee0e32c687f7721a499e716d56fd1985eccb3186fe81a2f8097b4bc1fb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
