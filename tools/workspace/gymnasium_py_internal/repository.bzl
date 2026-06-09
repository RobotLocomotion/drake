load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.3.0",
        sha256 = "acdcc261cb0145d6692d0cef69c1b30eb3b78982269c3795b8b3141060fc13fd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
