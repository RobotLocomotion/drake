load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def coinutils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CoinUtils",
        commit = "releases/2.11.9",
        sha256 = "15d572ace4cd3b7c8ce117081b65a2bd5b5a4ebaba54fadc99c7a244160f88b8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
