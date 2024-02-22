load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "a17f0666aa3e159a641c89fe1fe057a1b0cf4300",
        sha256 = "5031e7f22ba4fb8b88650809a59358d06cc996d842ea6ebfe67d83ca9a02b486",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
