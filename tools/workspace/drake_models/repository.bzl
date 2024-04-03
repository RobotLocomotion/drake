load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "52e7e2dac1dd19b5b0393e76e7af9f486b172595",
        sha256 = "368ccdc7f2e3742cc0015f6cb435181236ae3266c8b4fc88ce6e366f79437233",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
