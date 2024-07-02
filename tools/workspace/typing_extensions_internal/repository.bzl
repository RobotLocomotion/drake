load("//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.12.2",
        sha256 = "bf6f56b36d8bc9156e518eb1cc37a146284082fa53522033f772aefbecfd15fc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
