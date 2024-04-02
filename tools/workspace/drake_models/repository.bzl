load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "45c092654b1cd95b90d98f8c0b779250c27184ce",
        sha256 = "6c173c4fa2d371a94b4e71638b7c1ddb22e99b6a60516716789ad6de2b7c3e46",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
