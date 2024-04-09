load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "5c5fb727520ef2d5de146af01e7cbc57bfea43b2",
        sha256 = "b4cc4fa1558f0b2c7fe9dda80c489c3daa7f64190214a427b45743a3c28b9d29",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
