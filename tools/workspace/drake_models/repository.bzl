load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "8dc7d05b966900004219f696eb201ea6c9f0c1c7",
        sha256 = "960868443950f8bd4a9ff6a5f27fb68fed8fa6b4f53b77c7108bbef0c725b190",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
