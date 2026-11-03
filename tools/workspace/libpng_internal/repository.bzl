load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "pnggroup/libpng",
        upgrade_type = "tag",
        commit = "v1.6.59",
        sha256 = "2540302a1844ad2b2b501977abecfa850f265f97b78f065a712ab4074a89f5b5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
