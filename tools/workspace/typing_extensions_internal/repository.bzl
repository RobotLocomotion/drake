load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.9.0",
        sha256 = "7c510df94f5766765c3ed2498bec488881ab6b0b98be8b8dc9f47585f4b08897",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
