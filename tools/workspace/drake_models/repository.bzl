load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "6f54b46fbdaabeb3de206f1c9c19f84386503122",
        sha256 = "9195e5089b69a178e7625807957dc1a28fe3a2e13f251fe9a7a0b09ebb191fc5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
