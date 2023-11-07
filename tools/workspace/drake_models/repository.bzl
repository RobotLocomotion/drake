load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "fe5326c5ffc36fda12c58883d22d29dc86009d65",
        sha256 = "2a232f7da8d575fe1fcc6ad396f634d1adc1c9ee856168f011fbeb42a01c76b1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
