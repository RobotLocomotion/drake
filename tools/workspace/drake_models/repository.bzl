load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "a7f1dedbc7f14f1babe731a768ab7b123dd4f1c9",
        sha256 = "4cd0bb105095aafcea7f7701488b590b2a31d821118dc7bc1cb1f18a5b3b2817",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
