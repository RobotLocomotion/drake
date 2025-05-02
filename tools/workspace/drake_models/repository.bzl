load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "b1e6a452495c59eec34f849e76cc78f7631473c4",
        sha256 = "bfdd968d72e2b8bbba70c37bff53fdef32610b53a1c6c1032f6365254edea2a1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
