load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "268a68c6b7575068be48a54561e4aee19578bc94",
        sha256 = "078af2ba9fc61cb9c736d999c67212e2fa8815837442202c7a09a4ba9fc9a035",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
