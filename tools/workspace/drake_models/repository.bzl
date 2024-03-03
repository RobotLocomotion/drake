load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "5e9f85b6706aca1a45f79152c1e53e669f7a18b2",
        sha256 = "fcb1fdeb8ecbf199e1f7c1b9db6a90874ef7de63438aed376a28034276032b54",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
