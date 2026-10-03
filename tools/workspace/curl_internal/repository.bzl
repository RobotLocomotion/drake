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
        commit = "curl-8_22_0",
        sha256 = "222c6b5c1f368ac63aed59bce2774eb5def9e8e67e46e800be182e684d2845a3",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/schemes.patch",
        ],
        mirrors = mirrors,
    )
