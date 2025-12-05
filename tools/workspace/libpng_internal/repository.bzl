load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.51",
        sha256 = "b1872484c1c5c70acc79cbb15fb366df954fa8d5dacbe7f729d858902d17933c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
