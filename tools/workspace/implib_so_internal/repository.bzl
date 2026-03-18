load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "fc048a820528136160f1a4ca0ad512d094440156",
        sha256 = "ab4ccdcce7119461fa9e4f53e025cfa7a91f4ca7dae8f40648a1df93456214dc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
