load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "75011613676ba569f9af19147a21c9311da8cbe8",
        sha256 = "3f1cb854e1450b19c983bbf1323cf8482d5970bf1ee252ad00d86232b94a7a6c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
