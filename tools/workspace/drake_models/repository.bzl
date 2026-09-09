load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        upgrade_type = "commit",
        commit = "fa93453f12f50855d46ab1a4f94987ac2319a9ea",
        sha256 = "ce11f36b0fb58a7c50b750cbddf412f1a61bd679ba66c4e746d15ad26715ae8e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
