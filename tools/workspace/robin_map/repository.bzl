load("//tools/workspace:github.bzl", "github_archive")

def robin_map_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Tessil/robin-map",
        commit = "v1.4.0",
        sha256 = "7930dbf9634acfc02686d87f615c0f4f33135948130b8922331c16d90a03250c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
