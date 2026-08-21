load("//tools/workspace:github.bzl", "github_archive")

def gymnasium_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Farama-Foundation/Gymnasium",
        commit = "v1.2.3",
        sha256 = "b2a7f6d9c25eb0f36734429f856bc195e30e57d3226f0b1927d267054bd7a4bb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
