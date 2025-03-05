load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "2c961001bc919203c70722363e3be9ea390d98aa",
        sha256 = "fc028ede0b3ecbabaf8a7bb6ae5a4321bbb0b1d313f79108b9ff2b2114ac5c8c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
