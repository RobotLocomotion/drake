load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "5c3633d582ec923cdc1a032d48ef7053d4c34e18",
        sha256 = "96f9dd4075993015932e68e9c6b610ef96469533a82ded2e39544ca6a6642fa0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
