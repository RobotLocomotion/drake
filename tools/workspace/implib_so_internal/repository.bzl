load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "yugr/Implib.so",
        upgrade_type = "commit",
        commit = "a3e167cf978a5b8a38ec0f710d8c4331e8dcdd9a",
        sha256 = "c4fdcae8a6a7db45eb636e5b862be3da914e666d66a803944aedbeeefe0c70aa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
