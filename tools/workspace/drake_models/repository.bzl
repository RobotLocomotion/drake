load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "80a5d2609daaeb6f86a63545e6933e0edda06cda",
        sha256 = "40b7ce8894c75ab8ec047d897fe1b423c1f119d3af8b293fda2232ff72ac4a7a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
