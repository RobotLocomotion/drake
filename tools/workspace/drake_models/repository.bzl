load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "bca285305201523d5070da95bc52d3d0b65b28c4",
        sha256 = "5c3b730f5bd7d3318e95695c015b1f2c74a8bbfd23ef97f6298409adcf314711",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
