load("//tools/workspace:github.bzl", "github_archive")

def coinutils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CoinUtils",
        commit = "releases/2.11.10",
        sha256 = "80c7c215262df8d6bd2ba171617c5df844445871e9891ec6372df12ccbe5bcfd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
