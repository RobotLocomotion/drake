load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "e5d82a9b999e7be9944f4d116b4bd6f4aa4bead0",
        sha256 = "d3504944355c4ad01827218062f1d960789f207e70aec801f53f54d84ebfdfda",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
