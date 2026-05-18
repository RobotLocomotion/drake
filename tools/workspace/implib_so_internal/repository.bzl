load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "3a94ebf1b26e021cfa8c2202e4f4c7b31eab8eec",
        sha256 = "32aba7aed862d34b8b1e5353c2a69479a61b4e88b7e9388c634161827cb16a40",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
