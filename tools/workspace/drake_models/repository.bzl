load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "c18b846754cee4745ca389d33b60849bbf58314a",
        sha256 = "27b2c24f0ecb63ed9fbd0eebe65699ed6f979d7ed572f59c810d274c1dbd5d8f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
