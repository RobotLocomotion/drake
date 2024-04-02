load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "a8234918864d5510bd69cbb5e824b1a23991c204",
        sha256 = "1e78f567f60de83e77e7646bbe3c9b215b391f36325f8822fe0307f0f23f47f3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
