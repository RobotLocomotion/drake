load("//tools/workspace:github.bzl", "github_archive")

def coinutils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CoinUtils",
        commit = "releases/2.11.13",
        sha256 = "ddfea48e10209215748bc9f90a8c04abbb912b662c1aefaf280018d0a181ef79",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
