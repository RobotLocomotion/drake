load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "bc0447998cbba9046323581775d4ea7f5d00a1a2",
        sha256 = "64a166709c271a3a73fef617c907f940e207ec0ae5641debcfb2c0b1f51c40a6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
