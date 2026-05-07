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
        commit = "curl-8_20_0",
        sha256 = "738fe8ae973a6f171b4e7cf7146edd19894e19f09cd45a3b673ebdba3549a435",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/schemes.patch",
        ],
        mirrors = mirrors,
    )
