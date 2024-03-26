load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "6f49d40d6b24074e1e5153a4f0238c511cec549f",
        sha256 = "f0efbadbaa7cfe7681ecbc9336b0540b6ffd888335be33658e96eca66274539b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
