load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "cb72f82b5276cff8257ee13ab59f10327bf9f8f7",
        sha256 = "554bb278928e103c005ba2a491c78242a2742f717cda6427570e0e9288f5c84e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
