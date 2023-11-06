load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "140a1a28398ab162cf32dc8cfba397b03bc00269",
        sha256 = "b07d59a73000916db856ebb454f6808b26f4c9fc7c3477fdaf84375bf42ffffd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
