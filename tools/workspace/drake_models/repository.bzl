load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "140a1a28398ab162cf32dc8cfba397b03bc00269",
        sha256 = "823e8d3461da43473582c1b99c740cba5c305e576fb0324df8fa9e45047e0f98",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
