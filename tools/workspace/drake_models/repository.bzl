load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "93bae9efc792c1d1807650e330a29f58266a698b",
        sha256 = "5834ddc46519ef19c334a846c487490d440be7551bd813e7dd988b662e2efffd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
