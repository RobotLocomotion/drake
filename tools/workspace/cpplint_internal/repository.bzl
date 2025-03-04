load("//tools/workspace:github.bzl", "github_archive")

def cpplint_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cpplint/cpplint",
        commit = "f4363d7fc0d5f38c4fd41b658e069e96583da0d5",
        sha256 = "c1502a16e609c65731cd6eb62b94eb2e4ef5be195e57d85a5619f4ff12a0de3e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
