load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.10.0",
        sha256 = "a7306561f1ddf7bc00419b9f0d698d312a8eaa173b834e7c8e4ff32db5efd27f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
