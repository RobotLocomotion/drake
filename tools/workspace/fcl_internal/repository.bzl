load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "e5efcc41b57b2d0da3bf183480f1298a6d531f44",
        sha256 = "37aa84608083170329b6d9f9b07dc20d813b84d85546d1e3f1417cc8c2583c6e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
