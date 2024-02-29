load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "8dc7d05b966900004219f696eb201ea6c9f0c1c7",
        sha256 = "ac30930ac7a997010e4c0afde596da97cbf7ad2aa95d37d020473f17f820cbf0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
