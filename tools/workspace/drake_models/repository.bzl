load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        upgrade_type = "commit",
        commit = "4723170e1c8ee8bc59c342bb5c491ab8f263f1fb",
        sha256 = "f331abaf276753275b295d1ede0497c922d8ea93d3e8ba5017ce1d866cc2f396",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
