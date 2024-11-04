load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.0.0",
        sha256 = "d61a54c70d1960a0b64c63fe87edc30a0bac09551c27d8d94c36c54bf5fae5d9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
