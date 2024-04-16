load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "85c46d0fd9166e8ebf99c3f4e535020bd045d8de",
        sha256 = "daf7ca76bd3ae9f6c61d50a38460af3fdc4f11bd9d6e85a58c0642b3b9ac2603",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
