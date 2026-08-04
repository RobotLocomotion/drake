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
        commit = "curl-8_21_0",
        sha256 = "ec753aa6f408a3ca9f0d6d5f7a77417aecd1544db13c03ae5d443612bf367364",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/schemes.patch",
        ],
        mirrors = mirrors,
    )
