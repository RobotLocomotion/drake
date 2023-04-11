load("@drake//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "2d5f8a2d0fd49ae6021012f431b16fed4be4ce40",
        sha256 = "672d2d734a0c45167c022ce49e50c5d09b92b805b35e6022a735a821370b1052",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
