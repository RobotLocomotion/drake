load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "d2b82d335c4f19aa9b515b483c9d5e6dff0c8997",
        sha256 = "6b7390b95bc7ab50c6bbbf05a07efa727965485d71c8c5b34998cfec44dbbc27",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
