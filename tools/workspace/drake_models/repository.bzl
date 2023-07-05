load("@drake//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "90397c33cab9b7234d94c1018f2755bb9989c5a8",
        sha256 = "823e8d3461da43473582c1b99c740cba5c305e576fb0324df8fa9e45047e0f98",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
