load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "0d6d1a407c07ea21526013ec58e7a27c29cc5664",
        sha256 = "e87e71a72e41b8a52c815ea4795dc762332a0424e77d29fb58e6c99cc8762a2f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
