load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "b9d4f1cbe9e709e03e244539ffd9b51a04b3f734",
        sha256 = "a8e0a988f688f33a7a73d1477f85f979be6ba2f96f9f0e9179439260287e81e9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
