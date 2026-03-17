load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "0b45c6d097c26f51ddfc92357f989bc269fff40b",
        sha256 = "f0f67b17b9096d9063094cb8f037116e9bdaa07faad9e7ad2fe06abe9df0c35c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
