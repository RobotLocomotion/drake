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
        commit = "curl-8_18_0",
        sha256 = "be4b1e146ddd84bbb57f081e5c7238eee794e40563976ba2c89a44c433da219c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
