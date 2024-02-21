load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "92ed9246562caf6a77a4376b74b41eb777b319c8",
        sha256 = "99211fece3f4aec8ad93a93e5d8e0b3e76edf6bdc776335d7a4aec95c7dca004",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
