load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "6553d6d5298c392e669b61c84a3207560128cdae",
        sha256 = "128bd6287ad66ce7e7a1d4606c1e8ef7fb79c828d5093f98d735d5ca3ba66890",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
