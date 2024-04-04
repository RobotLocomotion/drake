load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "8820c49ebe20eefa8f11781db7c2e1bf3151dc3e",
        sha256 = "3b4cda078ccf96f2ac343fbe3b17b9b6698d67be6fffe944c044a043c337688d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
