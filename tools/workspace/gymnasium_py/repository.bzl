load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.0.0a2",
        sha256 = "8e05eb7b41aaa2c7cd2fca7fc8f380ca38a814a5fdc67d9ce8614e5014c90683",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
