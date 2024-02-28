load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "d0fe1a427a6bd39040ff3a77aaf6ddcc4d62a8fe",
        sha256 = "0a19e498d654e39dcf7e8f9144d06b3bbdd01a639cdc653a7d41cdc6d9ad5319",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
