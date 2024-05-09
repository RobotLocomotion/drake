load("//tools/workspace:github.bzl", "github_archive")

def coinutils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CoinUtils",
        commit = "releases/2.11.11",
        sha256 = "27da344479f38c82112d738501643dcb229e4ee96a5f87d4f406456bdc1b2cb4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
