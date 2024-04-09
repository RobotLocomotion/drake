load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "4662d1927764d26197e969d4bb9d9b189f9d774b",
        sha256 = "29c729bd2e46bd19f07c70e1332f6f60aa00ac008fd6632c6bdc4d4532d5ba57",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
