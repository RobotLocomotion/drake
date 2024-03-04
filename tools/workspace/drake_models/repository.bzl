load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "5e9f85b6706aca1a45f79152c1e53e669f7a18b2",
        sha256 = "b1d1188a487f61d4977644d4425fc0e39614ba0a05ef012974c80f393139f887",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
