load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        upgrade_type = "commit",
        commit = "0dcbab4240bc08f970729c7e9fe364a8b9681b31",
        sha256 = "ef027324af45936e31b7aad56156bf9d9ba44c8135dadb8ccc9f558816984df5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
