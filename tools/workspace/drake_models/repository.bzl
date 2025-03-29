load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "SeanCurtis-TRI/models",
        commit = "65f201d0c05e33e2d2ed0a225143aced5b7c63e2",
        sha256 = "7bf94408c3c2e2ca6553ea1121659f42b7e019f84b7b2776df5d25435b7317d5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
