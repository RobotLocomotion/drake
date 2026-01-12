load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "297a2ee67900e278077c467a9ed60bc7bea664cd",
        sha256 = "c754210bf0d63238049c97260af0f4f0c23cb3b96755df98f7d5474f55416fc1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
