load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        commit = "releases/3.14.13",
        sha256 = "2afcb057e7cf8ed7e07f50ee0a4a06d2e4d39e0f964777e9dd55fe56199a5e0a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
