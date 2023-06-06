load("@drake//tools/workspace:github.bzl", "github_archive")

def typing_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python/typing_extensions",
        commit = "4.6.3",
        sha256 = "14a7279d1e2b13d9f24acecab576e8c82a066173e5e620782a95e6856eca615e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
