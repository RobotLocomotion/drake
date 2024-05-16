load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "01e6a611ab37002658841c37f04fd6b9d7163e2f",
        sha256 = "7220234af6fff351b55873e021a489889aac1de0eaf4d0505889451350246a34",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
