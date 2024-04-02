load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "4e0a090a22def288a504e32aa55aa4837d3362e7",
        sha256 = "c44288baa4583c29a50504416c04e6c41e59fdf662ab73544be3d3892332ec8a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
