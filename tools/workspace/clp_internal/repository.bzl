load("//tools/workspace:github.bzl", "github_archive")

def clp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Clp",
        commit = "releases/1.17.9",
        sha256 = "b02109be54e2c9c6babc9480c242b2c3c7499368cfca8c0430f74782a694a49f",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/missing_include.patch",
        ],
        mirrors = mirrors,
    )
