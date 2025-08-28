load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.50",
        sha256 = "71158e53cfdf2877bc99bcab33641d78df3f48e6e0daad030afe9cb8c031aa46",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
