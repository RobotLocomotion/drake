load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "e9d2ff11dbdb52d2096a4c8b077a9b1cb564b790",
        sha256 = "b34501db762d9b778b41e1dd585eda7c9fe5af6c16b30495c1bc26fe5da9885e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
