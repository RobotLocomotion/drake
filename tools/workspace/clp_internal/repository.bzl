load("//tools/workspace:github.bzl", "github_archive")

def clp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Clp",
        commit = "releases/1.17.10",
        sha256 = "0d79ece896cdaa4a3855c37f1c28e6c26285f74d45f635046ca0b6d68a509885",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/missing_include.patch",
        ],
        mirrors = mirrors,
    )
