load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "dd09c0f96d0a7fdab346cc5535305ba60de442cb",
        sha256 = "6b70ee59eb6dd7412f524e3fc2e0a501e010433e00bbdcbaf44b9757e78c75ef",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
