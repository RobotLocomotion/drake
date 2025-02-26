load("//tools/workspace:github.bzl", "github_archive")

def cpplint_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cpplint/cpplint",
        commit = "f4363d7fc0d5f38c4fd41b658e069e96583da0d5",
        sha256 = "8bbba15797079b82f733a17b48f9da0c06d328cdaf14be12d37a0a5782d7be1a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
        ],
        mirrors = mirrors,
    )