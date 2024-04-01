load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "e4399227eb3d17cafcc47648b01b52adee4697a8",
        sha256 = "eb6a6ac4615d366cae1a02763400f7ffa67e5db0cef05c770125e1dc52a39a56",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
