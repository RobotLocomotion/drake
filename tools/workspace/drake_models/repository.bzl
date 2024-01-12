load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "0d7c2fc8ed155e2b954045097f28c111ed02e932",
        sha256 = "01ba7a26af69f098bd4b6e04e85f774c3a68ab4251e7275548a3eec96e81c4ce",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
