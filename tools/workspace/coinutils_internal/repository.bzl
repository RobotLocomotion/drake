load("//tools/workspace:github.bzl", "github_archive")

def coinutils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CoinUtils",
        upgrade_type = "release",
        commit = "4054af6f350ed5b432018e283ce6b0dbbb1ca13e",
        sha256 = "",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
