load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "c81f2458cf6d19a20a27e1495e7f07202536e845",
        sha256 = "1107e8314e49102a247f2e87666cba8bd1e76527112ce01d849e299cf8010d94",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
