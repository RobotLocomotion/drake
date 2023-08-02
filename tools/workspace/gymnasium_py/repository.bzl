load("@drake//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v0.29.0",
        sha256 = "3a108018cf50dc327bfc66e479c280b8112de597804ceeaa3cc50860a2905115",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
