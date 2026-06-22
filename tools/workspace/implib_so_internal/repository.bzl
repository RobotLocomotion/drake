load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "yugr/Implib.so",
        upgrade_type = "commit",
        commit = "30e547b2e8d608f7cd69bc8ec88d047034a40e35",
        sha256 = "0f5e111ce0648215a288c273639a045bde1f1b40139fbd85d06fd25e03af13a2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
